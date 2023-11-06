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

#include "BlindlyMoveToNextFruit.h"
#include "JointSpacePoint.h"
#include "JointSpacePath.h"
#include "moveit_state_tools.h"
#include "CollisionDetection.h"

// Make a CGAL convex hull.
using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_3 = K::Point_3;
using Surface_mesh = CGAL::Surface_mesh<Point_3>;
typedef CGAL::Surface_mesh_shortest_path_traits<K, Surface_mesh> Traits;
typedef CGAL::Surface_mesh_shortest_path<Traits> Surface_mesh_shortest_path;
using Primitive = CGAL::AABB_face_graph_triangle_primitive<Surface_mesh>;
using AABBTraits = CGAL::AABB_traits<K, Primitive>;

/**
 * @struct PathVisitor
 * @brief Visitor for Surface_mesh_shortest_path::shortest_path_sequence_to_source_points to build a robot path.
 */
struct PathVisitor {

	const Surface_mesh &mesh; ///< Reference to the triangle mesh
	Surface_mesh_shortest_path &path_algo; ///< Shortest path algorithm struct (for point lookups and such)
	std::vector<Surface_mesh_shortest_path::Face_location> &states; ///< The path being built.

	/**
	 * @brief Called when the path leaves a face through a half-edge.
	 * @param edge The half-edge for leaving the face (NOT the opposite half-edge where we enter the face!)
	 * @param t Interpolation value between the two vertices of the half-edge where the intersection occurs.
	 */
	void operator()(Surface_mesh_shortest_path::halfedge_descriptor edge, Surface_mesh_shortest_path::FT t) {
		states.push_back(path_algo.face_location(edge, t));
		states.push_back(path_algo.face_location(mesh.opposite(edge), 1.0 - t));
	}

	/**
	 * @brief Called when the path *exactly* crosses an edge.
	 * @param vertex The vertex of the edge where the path crosses.
	 */
	void operator()(Surface_mesh_shortest_path::vertex_descriptor vertex) {
		states.push_back(path_algo.face_location(vertex));
	}

	/**
	 * @brief Called when the path includes a point on the interior of a face.
	 * @param f The face where the path has a point.
	 * @param location The location of the point on the face (in barycentric coordinates)
	 */
	void operator()(Surface_mesh_shortest_path::face_descriptor f,
					Surface_mesh_shortest_path::Barycentric_coordinates location) {
		states.push_back({f, location});
	}
};

namespace mgodpl::planning {

	using namespace moveit_facade;


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
		if (this->plan) {

			std::cout << "We have a plan." << std::endl;

			// Check if we're at the first state in the plan.
			if (this->robot_model
						->distance(this->plan->path.front().joint_values.data(),
								   state.current_state.joint_values.data()) < 0.001) {

				std::cout << "We're at the first state in the plan." << std::endl;

				// Remove the first state in the plan, since we're there and we don't need it anymore.
				this->plan->path.erase(this->plan->path.begin());

				if (this->plan->path.empty()) {
					// If we're done with the plan. We've visited the apple! Remove it from the list.

					std::cout << "We're done with the plan." << std::endl;

					fruit_to_visit.erase(std::remove(fruit_to_visit.begin(), fruit_to_visit.end(), this->plan->target),
										 fruit_to_visit.end());

					std::cout << "Checking off target " << this->plan->target << std::endl;

					this->plan.reset();

				} else {

					// If we still have a plan, see if it's still valid.

					std::cout << "We still have a plan." << std::endl;

					for (size_t step = 0; step < this->plan->path.size(); ++step) {

						if (collision_detection.collides_ccd(
								step == 0 ? state.current_state : this->plan->path[step - 1], this->plan->path[step])) {

							std::cout << "Plan invalid after step " << step << std::endl;

							// If we collide, we need to replan.
							this->plan = std::nullopt;
							break;

						}

					}

					if (this->plan) {

						std::cout << "Plan still valid; returning next step." << std::endl;

						return this->plan->path.front();

					}

				}

			}

		}

		assert(!this->plan);

		std::cout << "We don't have a plan." << std::endl;

		// Build the Chull and an AABB thereof.

		Surface_mesh mesh = make_chull(fruit_to_visit);

		CGAL::AABB_tree<AABBTraits> tree{};

		Surface_mesh_shortest_path mesh_path(mesh);
		mesh_path.build_aabb_tree(tree);

		// Find the point on the chull closest to the current end-effector position.
		const auto &[face1, bary1] = mesh_path.locate(Point_3(ee_pos.x(), ee_pos.y(), ee_pos.z()), tree);

		const auto &pt1 = mesh_path.point(face1, bary1);
		const auto &nm1 = CGAL::Polygon_mesh_processing::compute_face_normal(face1, mesh);

		math::Vec3d pt1_vec(pt1.x(), pt1.y(), pt1.z());
		math::Vec3d nm1_vec(nm1.x(), nm1.y(), nm1.z());

		const JointSpacePoint &current_outside_tree = experiment_state_tools::robotStateFromPointAndArmvec(*robot_model,
																										   pt1_vec +
																										   nm1_vec.normalized(),
																										   -nm1_vec);

		// Keep trying to plan to one of the apples. If we can't, we'll just keep trying until we can or run out of apples.
		while (!fruit_to_visit.empty()) {

			// Find the closest fruit (Euclidean distance for now; we'll want to use the mesh path later).
			auto closest_fruit = std::min_element(fruit_to_visit.begin(),
												  fruit_to_visit.end(),
												  [&](const math::Vec3d &a, const math::Vec3d &b) {
													  return (a - ee_pos).squaredNorm() < (b - ee_pos).squaredNorm();
												  });

			std::cout << "Closest fruit: " << *closest_fruit << std::endl;

			// Find the closest point on the chull to the closest fruit.
			const auto &[face2, bary2] = mesh_path.locate(Point_3(closest_fruit->x(),
																  closest_fruit->y(),
																  closest_fruit->z()), tree);

			const auto &pt2 = mesh_path.point(face2, bary2);
			const auto &nm2 = CGAL::Polygon_mesh_processing::compute_face_normal(face2, mesh);

			math::Vec3d pt2_vec(pt2.x(), pt2.y(), pt2.z());
			math::Vec3d nm2_vec(nm2.x(), nm2.y(), nm2.z());

			const JointSpacePoint outside_tree = experiment_state_tools::robotStateFromPointAndArmvec(*robot_model,
																									  pt2_vec +
																									  nm2_vec.normalized(),
																									  -nm2_vec);

			// Geodesic path along the convex hull.
			mesh_path.add_source_point(face1, bary1);

			std::vector<Surface_mesh_shortest_path::Face_location> path;

			PathVisitor path_visitor{.mesh = mesh, .path_algo = mesh_path, .states = path};

			mesh_path.shortest_path_sequence_to_source_points(face2, bary2, path_visitor);

			JointSpacePoint at_target = outside_tree;

			// Move the end-effector to the closest fruit.
			experiment_state_tools::moveEndEffectorToPoint(*robot_model, at_target, *closest_fruit);

			if (collision_detection.collides_ccd(outside_tree, at_target)) {
				// We can't get there by a simple movement. We'll wanna do proper approach planning at some point.
				std::cout << "Can't get to fruit by simple movement." << std::endl;
				fruit_to_visit.erase(closest_fruit);
				continue;
			}

			// Let's build the following path:

			// 1. From the current state to the state on the chull.
			// 2. Along the chull to the state up close to the fruit.
			// 3. Go down into the tree up to the fruit.
			// 4. Go back up to the chull.

			moveit_facade::JointSpacePath jspath{.path = {current_outside_tree}};

			for (const auto &[face, barycentric]: path) {
				const auto &pt = mesh_path.point(face, barycentric);
				const auto &nm = CGAL::Polygon_mesh_processing::compute_face_normal(face, mesh);

				math::Vec3d pt_vec(pt.x(), pt.y(), pt.z());
				math::Vec3d nm_vec(nm.x(), nm.y(), nm.z());

				const JointSpacePoint &outside_tree = experiment_state_tools::robotStateFromPointAndArmvec(*robot_model,
																										   pt_vec +
																										   nm_vec.normalized(),
																										   -nm_vec);

				jspath.path.push_back(outside_tree);
			}

			jspath.path.push_back(at_target);
			jspath.path.push_back(outside_tree);

			this->plan = {.path = jspath.path, .target = *closest_fruit};

			std::cout << "Path length: " << jspath.path.size() << std::endl;

			return jspath.path.front();
		}

		std::cout << "No more fruit to visit." << std::endl;

		return std::nullopt;

	}
}