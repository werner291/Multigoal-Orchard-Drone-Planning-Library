// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/2/23.
//

#include <algorithm>
#include <moveit/robot_state/robot_state.h>
#include <CGAL/convex_hull_3.h>
#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include <CGAL/Surface_mesh_shortest_path/Surface_mesh_shortest_path.h>

#include "BlindlyMoveToNextFruit.h"
#include "JointSpacePoint.h"
#include "JointSpacePath.h"
#include "moveit_state_tools.h"
#include "CollisionDetection.h"
#include "../math/AABB.h"
#include "../math/AABBGrid.h"
#include "../visibility/GridVec.h"

#include "ConvexHullSpace.h"

//#include <ompl/base/StateSpace.h>
//
//namespace mgopdl::ompl_adapter {
//
//	namespace ob = ompl::base;
//
//	class OmplStateSpaceAdapter : public ob::StateSpace {
//
//		const moveit::core::RobotModelConstPtr &robot_model;
//
//	public:
//		unsigned int getDimension() const override {
//			return robot_model->getVariableCount();
//		}
//
//		double getMaximumExtent() const override {
//			return INFINITY;
//		}
//
//		double getMeasure() const override {
//			return INFINITY;
//		}
//
//		void enforceBounds(ompl::base::State *state) const override {
//			throw std::runtime_error("Not implemented.");
//		}
//
//		bool satisfiesBounds(const ompl::base::State *state) const override {
//			throw std::runtime_error("Not implemented.");
//		}
//
//		void copyState(ompl::base::State *destination, const ompl::base::State *source) const override {
//			throw std::runtime_error("Not implemented.");
//		}
//
//		double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override {
//			throw std::runtime_error("Not implemented.");
//		}
//
//		bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const override {
//			throw std::runtime_error("Not implemented.");
//		}
//
//		void interpolate(const ompl::base::State *from,
//						 const ompl::base::State *to,
//						 double t,
//						 ompl::base::State *state) const override {
//			throw std::runtime_error("Not implemented.");
//		}
//
//		ompl::base::StateSamplerPtr allocDefaultStateSampler() const override {
//			throw std::runtime_error("Not implemented.");
//		}
//
//		ompl::base::State *allocState() const override {
//			throw std::runtime_error("Not implemented.");
//		}
//
//		void freeState(ompl::base::State *state) const override {
//			throw std::runtime_error("Not implemented.");
//		}
//
//	};
//
//}

namespace mgodpl::planning {

	using namespace cgal;
	using namespace moveit_facade;
	using namespace experiment_state_tools;

	struct ApproachPath {
		Surface_mesh_shortest_path::Face_location face_location;
		JointSpacePath path;
	};

	std::optional<ApproachPath> goalStateThenOutsideApproachPlanning(random_numbers::RandomNumberGenerator &rng,
																	 const moveit::core::RobotModel &robot_model,
																	 const ConvexHullShellSpace &sp,
																	 const CollisionDetection &collision,
																	 const math::Vec3d &closest_fruit,
																	 const double max_target_distance,
																	 const JointSpacePoint &projected_outside_tree,
																	 const ConvexHullShellSpace::ShellPoint &fruit_projected_shellpoint,
																	 int goal_state_n,
																	 int shell_state_per_goal_state_n) {

		for (int attempt_i = 0; attempt_i < goal_state_n; ++attempt_i) {

			JointSpacePoint goal_state = randomUprightWithBaseNearState(robot_model,
																		1.0,
																		projected_outside_tree,
																		rng);
			moveEndEffectorNearPoint(robot_model, goal_state, closest_fruit, max_target_distance);

			if (!collision.collides(goal_state)) {
				std::cout << "Found a collision-free goal state after " << (attempt_i + 1) << " attempts." << std::endl;

				for (int attempt_j = 0; attempt_j < shell_state_per_goal_state_n; ++attempt_j) {
					ConvexHullShellSpace::ShellPoint nearby_point = sp.closestPoint(
							sp.to_carthesian(fruit_projected_shellpoint) + math::Vec3d(rng.uniformReal(-1.0, 1.0),
																					   rng.uniformReal(-1.0, 1.0),
																					   rng.uniformReal(-1.0, 1.0)));
					JointSpacePoint nearby_state = sp.shell_state(nearby_point);

					if (!collision.collides_ccd(nearby_state, goal_state)) {
						std::cout << "Succeeded after " << (attempt_j + 1) << " attempts." << std::endl;

						// The straight segment is collision-free.
						ApproachPath approach_path;
						approach_path.face_location = nearby_point;
						approach_path.path.path = {nearby_state, goal_state};
						return approach_path;
					}
				}
			}

		}

		return std::nullopt;

	}

#pragma GCC push_options
#pragma GCC optimize ("O0")
	std::optional<ApproachPath> customRRTApproachPlanning(random_numbers::RandomNumberGenerator &rng,
														  const moveit::core::RobotModel &robot_model,
														  const ConvexHullShellSpace &sp,
														  const CollisionDetection &collision,
														  const math::Vec3d &closest_fruit,
														  const double max_target_distance) {

		// Here, we work with an RRT rooted at the fruit.

		// First, project the fruit onto the convex hull.
		const auto &fruit_projected_shellpoint = sp.closestPoint(closest_fruit);
		// Then, get the shell state.
		const auto &projected_outside_tree = sp.shell_state(fruit_projected_shellpoint);

		// Then, project that down to the goal region.
		JointSpacePoint at_target = projected_outside_tree;
		moveEndEffectorNearPoint(robot_model, at_target, closest_fruit, max_target_distance);

		struct RRTNode {
			JointSpacePoint state;
			size_t parent;
		};

		const size_t GOAL_POINT = std::numeric_limits<size_t>::max();

		std::vector<RRTNode> tree_arena;

		if (!collision.collides(at_target)) {
			tree_arena.push_back({.state = at_target, .parent = GOAL_POINT});
		}

		const size_t ATTEMPTS = 1000;

		// first, let's try to get some goal samples:
		for (int iteration = 0; iteration < ATTEMPTS && tree_arena.size() < 10; ++iteration) {

			double aggressiveness = iteration / (double) ATTEMPTS;

			// Sample a random state near the at_target state.
			JointSpacePoint random_state = randomUprightWithBaseNearState(robot_model,
																		  aggressiveness,
																		  at_target,
																		  rng);

			// Try to project it down to the goal region.
			JointSpacePoint projected_state = random_state;
			moveEndEffectorNearPoint(robot_model, projected_state, closest_fruit, max_target_distance);

			if (!collision.collides(projected_state)) {
				// We found a goal state.
				tree_arena.push_back({.state = projected_state, .parent = GOAL_POINT});
			}

		}

		if (tree_arena.empty()) {
			std::cout << "No goal states found." << std::endl;
			return std::nullopt;
		} else {
			std::cout << "Found " << tree_arena.size() << " goal states." << std::endl;
		}

		const size_t RRT_ATTMEPTS = 1000;

		// Then, take 100 RRT samples.
		for (int iteration = 0; iteration < RRT_ATTMEPTS; ++iteration) {

			double aggressiveness = 2.0 * iteration / (double) RRT_ATTMEPTS;

			// Sample a random state near the at_target state.
			JointSpacePoint random_state = randomUprightWithBaseNearState(robot_model,
																		  aggressiveness,
																		  at_target,
																		  rng);

			// Find the closest existing node.
			auto closest = std::min_element(tree_arena.begin(),
											tree_arena.end(),
											[&](const auto &node1, const auto &node2) {
												return moveit_joint_distance(robot_model, node1.state, random_state) <
													   moveit_joint_distance(robot_model, node2.state, random_state);
											});

			{// Perform CCD TOI.
				std::optional<double> toi = collision.collision_ccd_toi(closest->state, random_state);

				if (!toi) {
					// We found a collision-free path.
					tree_arena.push_back({.state = random_state, .parent = (size_t) std::distance(tree_arena.begin(),
																								  closest)});
				} else {
					// At least grab the halfway point.
					JointSpacePoint halfway_state = interpolate(robot_model, closest->state, random_state, 0.5);

					if (!collision.collides(halfway_state)) {
						tree_arena.push_back({.state = halfway_state, .parent = (size_t) std::distance(tree_arena.begin(), closest)});
					}
				}
			}

			// Project the new node onto the hull.
			ConvexHullShellSpace::ShellPoint shell_point = sp.closestPoint(computeEndEffectorPosition(robot_model,
																									  tree_arena.back()
																											  .state));

			const auto shell_state = sp.shell_state(shell_point);

			// Check the CCD.
			std::optional<double> toi = collision.collision_ccd_toi(shell_state, tree_arena.back().state);

			if (!toi) {

				// We're out! Trace the path back and return it.

				ApproachPath approach_path;

				approach_path.face_location = shell_point;

				size_t current = tree_arena.size() - 1;

				while (current != GOAL_POINT) {
					assert(current < tree_arena.size());
					assert(tree_arena[current].state.joint_values.size() == 11);
					approach_path.path.path.push_back(tree_arena[current].state);
					current = tree_arena[current].parent;
				}

				return approach_path;

			} else {
				// At least grab the halfway point.
				JointSpacePoint halfway_state = interpolate(robot_model,
															sp.shell_state(shell_point),
															tree_arena.back().state,
															0.5);

				if (!collision.collides(halfway_state)) {
					tree_arena.push_back({.state = halfway_state, .parent = (size_t) std::distance(tree_arena.begin(),
																								   closest)});
				}
			}


		}

		return std::nullopt;

	}
#pragma GCC pop_options


	/**
	 * Plan a single approach path from outside the convex hull shell to fruit.
	 *
	 * @param collision		The collision detection object.
	 * @param fruit			The fruit to approach.
	 * @param mesh			The convex hull mesh.
	 * @param mesh_path		The convex hull mesh path algorithm struct, which we abuse to compute the face locations.
	 *
	 * @return The approach path, or std::nullopt if no path could be found.
	 */
	std::optional<ApproachPath> plan_single_approach_path(const moveit::core::RobotModel &robot_model,
														  const ConvexHullShellSpace &sp,
														  const CollisionDetection &collision,
														  const math::Vec3d &closest_fruit,
														  const double max_target_distance) {

//		// First, perform a naive projection to get a rough idea of where the path should go.
//		const auto &fruit_projected_shellpoint = sp.closestPoint(closest_fruit);
//		const auto &projected_outside_tree = sp.shell_state(fruit_projected_shellpoint);
//
//		// Project the shell state onto the goal region.
//		JointSpacePoint at_target = projected_outside_tree;
//		moveEndEffectorNearPoint(robot_model, at_target, closest_fruit, max_target_distance);
//
//		if (!collision.collides_ccd(projected_outside_tree, at_target)) {
//			// The straight segment is collision-free.
//			ApproachPath approach_path;
//			approach_path.face_location = fruit_projected_shellpoint;
//			approach_path.path.path = {projected_outside_tree, at_target};
//
//			std::cout << "Straight segment is collision-free." << std::endl;
//
//			return approach_path;
//		}

		random_numbers::RandomNumberGenerator rng;

		return customRRTApproachPlanning(rng,
										 robot_model,
										 sp,
										 collision,
										 closest_fruit,
										 max_target_distance);

//		return goalStateThenOutsideApproachPlanning(rng,
//													robot_model,
//													sp,
//													collision,
//													closest_fruit,
//													max_target_distance,
//													projected_outside_tree,
//													fruit_projected_shellpoint,
//													100,
//													100);

	}


	std::optional<JointSpacePoint>
	BlindlyMoveToNextFruit::nextMovement(const mgodpl::experiments::VoxelShroudedSceneInfoUpdate &state) {

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

		// Get the end-effector position.
		math::Vec3d ee_pos = computeEndEffectorPosition(*robot_model, state.current_state);

		// At this point, our goal is to, starting from the position that the robot is currently in (state.current_state),
		// to bring the end-effector close to each of the fruit in fruit_to_visit.

		// First, let's see if our current plans are still valid.

		CollisionDetection collision_detection({state.filtered_tree_meshes.leaves_mesh,
												state.filtered_tree_meshes.trunk_mesh}, robot_model);

		// Check if we even have a plan.
		if (!this->plan.empty()) {

			// Take and delete the front point in the plan.
			const PlanState &plan_state = plan.front();

			// Check if we're at the first state in the plan.
			if (!plan_state.point.significantly_different_from(state.current_state.joint_values, 1e-6)) {

				// If we were supposed to go to a fruit check it off. (assert to make sure we're actually at the fruit).
				if (plan_state.target) {
					assert((ee_pos - *plan_state.target).squaredNorm() < this->max_target_distance + 1e-6);
					auto itr = std::find(fruit_to_visit.begin(), fruit_to_visit.end(), *plan_state.target);
					assert(itr != fruit_to_visit.end());
					fruit_to_visit.erase(itr);
					on_target_reached(*plan_state.target);
				}

				this->plan.erase(this->plan.begin());

			}
		}

		if (!current_path_is_collision_free(state.current_state, collision_detection)) {
			this->plan.clear();
		}

		if (!this->plan.empty()) {
			return this->plan.front().point;
		}

		const std::vector<math::Vec3d> leaf_vertices =
				state.filtered_tree_meshes.leaves_mesh.vertices | ranges::views::transform([](const auto &vertex) {
					return math::Vec3d(vertex.x, vertex.y, vertex.z);
				}) | ranges::to<std::vector>();

		// Build the Chull and an AABB thereof.
		ConvexHullShellSpace sp(leaf_vertices, robot_model);

		// Find the point on the chull closest to the current end-effector position.
		const ConvexHullShellSpace::ShellPoint current_state_sp = sp.closestPoint(ee_pos);

		std::vector<double> distances = sp.distances_to_many(current_state_sp,
															 fruit_to_visit |
															 ranges::views::transform([&](const auto &fruit) {
																 return sp.closestPoint(fruit);
															 }) | ranges::to<std::vector>());

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

			const auto &approach_path = plan_single_approach_path(*robot_model,
																  sp,
																  collision_detection,
																  closest_fruit,
																  this->max_target_distance);

			if (!approach_path) {
				std::cout << "No approach path found." << std::endl;
				on_target_rejected(closest_fruit);
				fruit_to_visit.erase(fruit_to_visit.begin() + (long) closest_fruit_index);
				distances.erase(distances.begin() + (long) closest_fruit_index);
				std::cout << "Rejecting fruit at " << closest_fruit << std::endl;
				continue;
			}

			for (const auto &shell_point: sp.path_along_shell(current_state_sp, approach_path->face_location)) {
				this->plan.push_back({.point = sp.shell_state(shell_point), .target = std::nullopt});
			}

			for (size_t approach_path_i = 0; approach_path_i + 1 < approach_path->path.path.size(); ++approach_path_i) {
				this->plan.push_back({.point = approach_path->path.path[approach_path_i], .target = std::nullopt});
			}

			this->plan.push_back({.point = approach_path->path.path.back(), .target = closest_fruit});

			for (size_t approach_path_i = approach_path->path.path.size(); approach_path_i > 0; --approach_path_i) {
				this->plan.push_back({.point = approach_path->path.path[approach_path_i - 1], .target = std::nullopt});
			}

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
			while (jump_to + 1 < plan.size() && // Jump at most as far as the last step; always keep the last step
				   !plan[jump_to].target && // Do not jump further than a state that has a fruit.
				   !collision_detection.collides_ccd(from_state,
													 plan[jump_to +
														  1].point)) { // Only jump if the motion is collision-free.
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

			for (const double aggressiveness: {0.1, 0.2, 0.5}) {
				// Then, a point halfway between the midpoint and the halfway point.
				JointSpacePoint halfway_midpoint_state = interpolate(*robot_model,
																	 halfway_state,
																	 midpoint_state,
																	 aggressiveness);

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