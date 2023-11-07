// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/2/23.
//

#include <algorithm>
#include <moveit/robot_state/robot_state.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/measure.h>

#include "BlindlyMoveToNextFruit.h"
#include "JointSpacePoint.h"
#include "JointSpacePath.h"
#include "moveit_state_tools.h"
#include "CollisionDetection.h"
#include "../math/AABB.h"
#include "../math/AABBGrid.h"
#include "../visibility/GridVec.h"
#include "cgal_chull_shortest_paths.h"

namespace mgodpl::planning {

	using namespace moveit_facade;
	using namespace cgal;

	Surface_mesh make_chull(const std::vector<math::Vec3d> &points) {
		Surface_mesh mesh;

		std::vector<Point_3> cgal_points;

		// Add all the leaf vertices.
		for (const auto &point: points) {
			cgal_points.emplace_back(point.x(), point.y(), point.z());
		}

		// Grab the convex hull.
		CGAL::convex_hull_3(cgal_points.begin(), cgal_points.end(), mesh);

		return mesh;
	}


	std::optional<JointSpacePoint>
	BlindlyMoveToNextFruit::nextMovement(const mgodpl::experiments::VoxelShroudedSceneInfoUpdate &state) {

		std::cout << "BlindlyMoveToNextFruit::nextMovement" << std::endl;

		/*
		 * We have new information about the scene. We need to return the next state that we wish to immediately go to.
		 *
		 * All information we have at this point might be stale. We'll want to check if it's still valid and, if so,
		 * if it's still desirable to use.
		 */

		// Add the newly detected fruit to the list of fruit to visit.
		fruit_to_visit.insert(fruit_to_visit.end(),
							  state.newly_detected_fruits.begin(),
							  state.newly_detected_fruits.end());

		std::cout << "Storing " << state.newly_detected_fruits.size() << " fruits." << std::endl;


		// Get the end-effector position.
		math::Vec3d ee_pos = computeEndEffectorPosition(*robot_model, state.current_state);

		// At this point, our goal is to, starting from the position that the robot is currently in (state.current_state),
		// to bring the end-effector close to each of the fruit in fruit_to_visit.

		// First, let's see if our current plans are still valid.

		CollisionDetection collision_detection({state.filtered_tree_meshes.leaves_mesh,
												state.filtered_tree_meshes.trunk_mesh}, robot_model);

		// Check if we even have a plan.
		if (!this->plan.empty()) {

			std::cout << "We have a plan." << std::endl;

			// Take and delete the front point in the plan.
			const PlanState &plan_state = plan.front();

			// Check if we're at the first state in the plan.
			if (!plan_state.point.significantly_different_from(state.current_state.joint_values, 1e-6)) {

				std::cout << "We're at the first state in the plan." << std::endl;

				// If we were supposed to go to a fruit check it off. (assert to make sure we're actually at the fruit).
				if (plan_state.target) {
					assert((ee_pos - *plan_state.target).squaredNorm() < this->max_target_distance + 1e-6);
					auto itr = std::find(fruit_to_visit.begin(), fruit_to_visit.end(), *plan_state.target);
					if(itr == fruit_to_visit.end()) {
						std::cout << "Fruit at " << *plan_state.target << " was already checked off?!" << std::endl;
						abort();
					}
					fruit_to_visit.erase(itr);
					std::cout << "Checked off fruit at " << *plan_state.target << std::endl;
					on_target_reached(*plan_state.target);

				}

				this->plan.erase(this->plan.begin());

			}
		}

		if (!current_path_is_collision_free(state.current_state, collision_detection)) {
			std::cout << "Current path is not collision-free; clearing." << std::endl;
			this->plan.clear();
		}

		if (!this->plan.empty()) {
			std::cout << "Returning next state in plan." << std::endl;
			return this->plan.front().point;
		}

		// Build the Chull and an AABB thereof.

		Surface_mesh mesh = make_chull(fruit_to_visit);

		CGAL::AABB_tree<AABBTraits> tree{};

		Surface_mesh_shortest_path mesh_path(mesh);
		mesh_path.build_aabb_tree(tree);

		// Find the point on the chull closest to the current end-effector position.
		const auto &[face1, bary1] = mesh_path.locate(Point_3(ee_pos.x(), ee_pos.y(), ee_pos.z()), tree);

		// Geodesic path along the convex hull.
		mesh_path.add_source_point(face1, bary1);

		const auto &pt1 = mesh_path.point(face1, bary1);
		const auto &nm1 = CGAL::Polygon_mesh_processing::compute_face_normal(face1, mesh);

		math::Vec3d pt1_vec(pt1.x(), pt1.y(), pt1.z());
		math::Vec3d nm1_vec(nm1.x(), nm1.y(), nm1.z());

		const JointSpacePoint &current_outside_tree = experiment_state_tools::robotStateFromPointAndArmvec(*robot_model,
																										   pt1_vec +
																										   nm1_vec.normalized(),
																										   -nm1_vec);

		std::vector<double> distances;
		distances.reserve(fruit_to_visit.size());

		// Get the geodesic distance to all the fruit.
		for (const auto &fruit: fruit_to_visit) {

			// Find the closest point on the chull to the closest fruit.
			const auto &[face2, bary2] = mesh_path.locate(Point_3(fruit.x(), fruit.y(), fruit.z()), tree);

			double distance = mesh_path.shortest_distance_to_source_points(face2, bary2).first;

			distances.push_back(distance);

		}

		// Keep trying to plan to one of the apples. If we can't, we'll just keep trying until we can or run out of apples.
		while (!fruit_to_visit.empty()) {

			size_t closest_fruit_index = 0;
			double closest_fruit_distance = distances[0];

			for (size_t i = 1; i < fruit_to_visit.size(); ++i) {
				if (distances[i] < closest_fruit_distance) {
					closest_fruit_index = i;
					closest_fruit_distance = distances[i];
				}
			}

			math::Vec3d closest_fruit = fruit_to_visit[closest_fruit_index];

			std::cout << "Closest fruit: " << closest_fruit << std::endl;

			// Find the closest point on the chull to the closest fruit.
			const auto &[face2, bary2] = mesh_path.locate(Point_3(closest_fruit.x(),
																  closest_fruit.y(),
																  closest_fruit.z()), tree);

			const auto &pt2 = mesh_path.point(face2, bary2);
			const auto &nm2 = CGAL::Polygon_mesh_processing::compute_face_normal(face2, mesh);

			math::Vec3d pt2_vec(pt2.x(), pt2.y(), pt2.z());
			math::Vec3d nm2_vec(nm2.x(), nm2.y(), nm2.z());

			const JointSpacePoint outside_tree = experiment_state_tools::robotStateFromPointAndArmvec(*robot_model,
																									  pt2_vec +
																									  nm2_vec.normalized(),
																									  -nm2_vec);

			double max_distance_from_straight = INFINITY;
			JointSpacePoint at_target_valid = outside_tree;

			experiment_state_tools::moveEndEffectorNearPoint(*robot_model,
															 at_target_valid,
															 closest_fruit,
															 max_target_distance);

			if (!collision_detection.collides_ccd(current_outside_tree, outside_tree)) {
				max_distance_from_straight = moveit_joint_distance(*robot_model, at_target_valid, outside_tree);
			} else {

				for (int goal_sample_i = 0; goal_sample_i < 1000; ++goal_sample_i) {

					// Generate a random state.
					JointSpacePoint goal_state = experiment_state_tools::randomUprightWithBaseNearState(*robot_model,
																										0.5,
																										at_target_valid,
																										goal_sample_i);

					// Move the end-effector to the closest fruit.
					experiment_state_tools::moveEndEffectorNearPoint(*robot_model,
																	 goal_state,
																	 closest_fruit,
																	 max_target_distance);

					double distance = moveit_joint_distance(*robot_model, outside_tree, goal_state);

					if (distance < max_distance_from_straight &&
						!collision_detection.collides_ccd(goal_state, outside_tree)) {

						max_distance_from_straight = distance;
						at_target_valid = goal_state;

					}

				}
			}

			if (!finite(max_distance_from_straight)) {
				std::cout << "Can't get to fruit at " << closest_fruit << std::endl;
				on_target_rejected(closest_fruit);
				fruit_to_visit.erase(fruit_to_visit.begin() + (long) closest_fruit_index);
				distances.erase(distances.begin() + (long) closest_fruit_index);
				continue;
			}

			std::vector<Surface_mesh_shortest_path::Face_location> path;

			PathVisitor path_visitor{.mesh = mesh, .path_algo = mesh_path, .states = path};

			mesh_path.shortest_path_sequence_to_source_points(face2, bary2, path_visitor);

			std::reverse(path.begin(), path.end());

			// Let's build the following path:

			// 1. From the current state to the state on the chull.
			// 2. Along the chull to the state up close to the fruit.
			// 3. Go down into the tree up to the fruit.
			// 4. Go back up to the chull.

			std::vector<PlanState> path_states{{.point = current_outside_tree, .target = std::nullopt},};

			for (const auto &[face, barycentric]: path) {
				const auto &pt = mesh_path.point(face, barycentric);
				const auto &nm = CGAL::Polygon_mesh_processing::compute_face_normal(face, mesh);

				math::Vec3d pt_vec(pt.x(), pt.y(), pt.z());
				math::Vec3d nm_vec(nm.x(), nm.y(), nm.z());

				const JointSpacePoint &outside_tree = experiment_state_tools::robotStateFromPointAndArmvec(*robot_model,
																										   pt_vec +
																										   nm_vec.normalized(),
																										   -nm_vec);

				path_states.push_back({.point = outside_tree, .target = std::nullopt});
			}

			path_states.push_back({.point = outside_tree, .target = {}});
			path_states.push_back({.point = at_target_valid, .target = closest_fruit});
			path_states.push_back({.point = outside_tree, .target = {}});

			this->plan = path_states;

			localOptimizeCurrentPlan(state.current_state, collision_detection);

			// Sanity check: make sure the path is collision-free.
			if (!current_path_is_collision_free(state.current_state, collision_detection)) {
				// TODO: if this fails, it's probably due to asymmetries in the CCD collision checking.
				std::cout << "Computed path is not collision-free." << std::endl;
				this->plan.clear();
				on_target_rejected(closest_fruit);
				fruit_to_visit.erase(fruit_to_visit.begin() + (long) closest_fruit_index);
				distances.erase(distances.begin() + (long) closest_fruit_index);
				std::cout << "Rejecting fruit at " << closest_fruit << std::endl;
				continue;
			}

			return this->plan.front().point;
		}

		std::cout << "No more fruit to visit." << std::endl;

		return std::nullopt;

	}

	bool
	BlindlyMoveToNextFruit::current_path_is_collision_free(const moveit_facade::JointSpacePoint &robot_current_state,
														   const moveit_facade::CollisionDetection &collision_detection) {

		if (this->plan.empty()) {
			return true;
		}

		if (collision_detection.collides_ccd(robot_current_state, this->plan.front().point)) {
			return false;
		}

		for (size_t step = 0; step + 1 < this->plan.size(); ++step) {
			if (collision_detection.collides_ccd(this->plan[step].point, this->plan[step + 1].point)) {
				return false;
			}
		}

		return true;

	}

	void BlindlyMoveToNextFruit::localOptimizeCurrentPlan(const JointSpacePoint &robot_current_state,
														  const CollisionDetection &collision_detection) {

		// An empty plan is as short as it gets; no need to do anything.
		if (this->plan.empty()) {
			return;
		}

		shortcutBySkipping(robot_current_state, collision_detection);
		optimizeByMidpointPulling(robot_current_state, collision_detection);

	}

	void BlindlyMoveToNextFruit::shortcutBySkipping(const JointSpacePoint &robot_current_state,
													const CollisionDetection &collision_detection) {

		// Try to find pairs of states in the plan where the motion between them is collision-free,
		// such that we can skip all intermediate states. As a rule, we may not skip states where
		// the robot's end-effector is at a fruit, or we'll end up skipping the fruit.
		for (size_t segment_i = 0; segment_i < plan.size(); ++segment_i) {
			// Iterate over all steps in the plan (count the current state as step 0).

			// The start of the segment.
			const JointSpacePoint &from_state = segment_i == 0 ? robot_current_state : plan[segment_i - 1].point;

			// Store the end of the segment as the first jump candidate. If it remains the same, we won't jump.
			size_t jump_to = segment_i;

			// Then, we'll keep increasing it as long as we maintain a valid jump.
			while (
					jump_to + 1 < plan.size() && // Jump at most as far as the last step; always keep the last step
					!plan[jump_to].target && // Do not jump further than a state that has a fruit.
					!collision_detection.collides_ccd(from_state, plan[jump_to + 1].point)) { // Only jump if the motion is collision-free.
				++jump_to;
			}

			// If we didn't jump, we're done with this segment.
			if (jump_to == segment_i) {
				continue;
			}

			std::cout << "Jumping from " << segment_i << " to " << jump_to << std::endl;

			// We can jump from step_i to jump_to; we do so by deleting all intermediate steps.
			plan.erase(plan.begin() + (long) segment_i, plan.begin() + (long) jump_to);

		}
	}

	void BlindlyMoveToNextFruit::optimizeByMidpointPulling(const JointSpacePoint &robot_current_state,
														   const CollisionDetection &collision_detection) {

		// Find triplets of states.
		for (size_t triplet_midpoint = 0; triplet_midpoint + 1 < plan.size(); ++triplet_midpoint) {

			// The start of the segment.
			const JointSpacePoint &from_state =
					triplet_midpoint == 0 ? robot_current_state : plan[triplet_midpoint - 1].point;

			// The end of the segment.
			const JointSpacePoint &to_state = plan[triplet_midpoint + 1].point;

			// The midpoint of the segment.
			const JointSpacePoint &midpoint_state = plan[triplet_midpoint].point;

			// Get a point halfway between from_state and to_state.
			JointSpacePoint halfway_state = interpolate(*robot_model, from_state, to_state, 0.5);

			for (const double aggressiveness : {0.1, 0.2, 0.5}) {
				// Then, a point halfway between the midpoint and the halfway point.
				JointSpacePoint halfway_midpoint_state = interpolate(*robot_model, halfway_state, midpoint_state, aggressiveness);

				// If it's an at-target state, reproject it.
				if (plan[triplet_midpoint].target) {
					experiment_state_tools::moveEndEffectorNearPoint(*robot_model,
																	 halfway_midpoint_state,
																	 *plan[triplet_midpoint].target,
																	 max_target_distance);
				}

				// If it's collision-free, we can replace the midpoint with the halfway midpoint.
				if (!collision_detection.collides_ccd(from_state, halfway_midpoint_state) &&
					!collision_detection.collides_ccd(halfway_midpoint_state, to_state)) {

					plan[triplet_midpoint].point = halfway_midpoint_state;

				}
			}
		}
	}
}