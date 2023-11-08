// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 8-11-23.
//

#ifndef MGODPL_CONVEXHULLSPACE_H
#define MGODPL_CONVEXHULLSPACE_H

#include "moveit_forward_declarations.h"
#include "cgal_chull_shortest_paths.h"
#include "../math/Vec3.h"
#include "JointSpacePoint.h"

namespace mgodpl {

	/**
	 * Represents an embedding of a convex hull into the robot's joint space, effectively
	 * defined by the embedding function ConvexHullShellSpace::shell_state.
	 */
	class ConvexHullShellSpace {

		/// The mesh representing the convex hull.
		mgodpl::cgal::Surface_mesh mesh;

		/// The shortest path algorithm on the mesh (I'm considering whether this should be a member at all).
		mgodpl::cgal::Surface_mesh_shortest_path mesh_path;

		/// The AABB tree for the mesh.
		CGAL::AABB_tree<mgodpl::cgal::AABBTraits> tree;

		/// The robot model.
		moveit::core::RobotModelConstPtr robot_model;

	public:

		using ShellPoint = cgal::Surface_mesh_shortest_path::Face_location;

		/**
		 * Buidl a shell space based on the convex hull around the given set of points.
		 *
		 * @param points 	The points to build the convex hull around.
		 */
		ConvexHullShellSpace(const std::vector<math::Vec3d> &points,
							 const moveit::core::RobotModelConstPtr &robotModel);

		/**
		 * Find the closest point on the convex hull to the given point.
		 *
		 * @param point	The point to find the closest point on the convex hull to.
		 * @return	The closest point on the convex hull to the given point.
		 */
		ShellPoint closestPoint(const math::Vec3d &point) const;

		/**
		 * Get the robot state corresponding to the given point on the convex hull.
		 *
		 * Specifically, this is: a robot state (expressed as joint values) that has the end-effector
		 * at the given point on the convex hull, and the arm vector pointing at the convex hull.
		 *
		 * This function effectively defines the embedding of the convex hull into the robot's joint space.
		 *
		 * @param sp 		The point on the convex hull to get the robot state for.
		 * @return 			The robot state corresponding to the given point on the convex hull.
		 */
		moveit_facade::JointSpacePoint shell_state(const ShellPoint &sp) const;

		/**
		 * Get the distances from the given point on the convex hull to many other points on the convex hull.
		 *
		 * Note: this is far more efficient than doing this pairwise, since the shortest path algorithm
		 * computes a sequence tree that can be reused for many points from a single source point.
		 *
		 * @param sp 					The point on the convex hull to get the distances from.
		 * @param other_points 			The other points on the convex hull to get the distances to.
		 * @return 						The distances from the given point to the other points.
		 */
		std::vector<double> distances_to_many(const ShellPoint &sp, const std::vector <ShellPoint> &other_points) const;

		/**
		 * Get the shortest path along the convex hull between the two given points, returned as a sequence of points.
		 * Note: for every edge traversal, we have two points: one on the previous face, and one on the next face.
		 *
		 * @param start 		The start point.
		 * @param end 			The end point.
		 * @return 				The shortest path between the two points.
		 */
		std::vector <ShellPoint> path_along_shell(const ShellPoint &start, const ShellPoint &end) const;

		math::Vec3d to_carthesian(const ShellPoint &sp) const;

	};
}

#endif //MGODPL_CONVEXHULLSPACE_H
