module;

// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#include "RobotState.h"
#include "cgal_chull_shortest_paths.h"
#include "RobotModel.h"
#include "state_tools.h"

#include <memory>
#include <functional>
#include <CGAL/Side_of_triangle_mesh.h>

export module shell_state_projection;

using namespace mgodpl;

export namespace mgodpl {

	/**
	 * @brief Projects a given goal state to a shell state.
	 *
	 * This function finds the point on the convex hull shell closest to the base translation of the given state,
	 * and then uses fromEndEffectorAndVector to create a shell state on that surface.
	 *
	 * @param to_project The goal state to be projected.
	 * @param tree_convex_hull The convex hull of the tree.
	 * @param robot The robot model used for the projection.
	 * @return The idealized shell state.
	 */
	RobotState project_to_shell_state(const RobotState &to_project,
									  const cgal::CgalMeshData &tree_convex_hull,
									  const robot_model::RobotModel &robot) {
		// Find the surface point on the tree closest to the goal:
		const auto surface_pt = mgodpl::cgal::from_face_location(
				mgodpl::cgal::locate_nearest(to_project.base_tf.translation, tree_convex_hull),
				tree_convex_hull);

		// Compute an idealized shell state. (This is the same as the straight-in starting state.)
		return fromEndEffectorAndVector(robot, surface_pt.surface_point, surface_pt.normal);
	}

	/**
	 * @brief Create an acceptance function for whether a given state is considered "outside" the tree.
	 * @param convex_hull 	The convex hull of the tree.
	 * @return A function that returns true if the state is outside the tree, false otherwise.
	 */
	std::function<bool(const RobotState &)> accept_outside_tree(const cgal::CgalMeshData &convex_hull) {
		auto inside = std::make_shared<CGAL::Side_of_triangle_mesh<cgal::Surface_mesh, cgal::K>>(convex_hull.convex_hull);
		return [inside = std::move(inside)](const RobotState &state) {
			// TODO: this is technically not 100% accurate but if we get to this point the planning problem is trivial.
			// Also, for our analysis this doesn't matter: this only makes the problem easier for RRT, and harder for our method.
			return (*inside)(cgal::to_cgal_point(state.base_tf.translation)) ==
				   CGAL::ON_UNBOUNDED_SIDE;
		};
	};
}