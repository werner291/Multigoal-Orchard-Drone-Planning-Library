/**
 * @file enclosing_sphere.h
 * @brief This file contains utility functions for calculating the smallest enclosing sphere around 3D points.
 */

#ifndef NEW_PLANNERS_ENCLOSING_SPHERE_H
#define NEW_PLANNERS_ENCLOSING_SPHERE_H

#include <geometric_shapes/bodies.h>
#include <geometry_msgs/geometry_msgs/msg/point.hpp>
#include "../AppleTreePlanningScene.h"

namespace utilities {

	/**
	 * @brief Computes an enclosing sphere around the leaves of an apple tree.
	 *
	 * @param planning_scene_message The planning scene message that contains information about the apple tree.
	 * @param padding An additional radius to add to the size of the enclosing sphere.
	 * @return A bounding sphere that encloses the leaves of the apple tree.
	 */
	bodies::BoundingSphere
	compute_enclosing_sphere_around_leaves(const moveit_msgs::msg::PlanningScene &planning_scene_message,
										   const double padding);

	/**
	 * @brief Computes the smallest enclosing sphere around a set of 3D points.
	 *
	 * @param points_eigen A vector of 3D points for which the enclosing sphere is computed.
	 * @param padding An additional radius to add to the size of the enclosing sphere.
	 * @return A bounding sphere that encloses the input 3D points.
	 */
	bodies::BoundingSphere compute_enclosing_sphere_around_points(const std::vector<Eigen::Vector3d> &points_eigen,
																  const double padding = 0.0);

	/**
	 * @brief Extracts the vertices of the leaves from the apple tree planning scene.
	 *
	 * @param scene_info The apple tree planning scene information.
	 * @return A vector of 3D points that represent the vertices of the leaves.
	 */
	std::vector<geometry_msgs::msg::Point> extract_leaf_vertices(const AppleTreePlanningScene &scene_info);

	/**
	 * @brief Extracts the vertices of the leaves from a simplified orchard.
	 *
	 * @param orchard The simplified orchard from which the vertices of the leaves are extracted.
	 * @return A vector of 3D points that represent the vertices of the leaves.
	 */
	std::vector<geometry_msgs::msg::Point> extract_leaf_vertices(const SimplifiedOrchard &orchard);
}

#endif //NEW_PLANNERS_ENCLOSING_SPHERE_H
