//
// Created by werner on 6-3-23.
//

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
	 * @param padding A value to add to the size of the enclosing sphere.
	 * @return The bounding sphere that encloses the leaves of the apple tree.
	 */
	bodies::BoundingSphere
	compute_enclosing_sphere_around_leaves(const moveit_msgs::msg::PlanningScene &planning_scene_message,
										   const double padding);

	bodies::BoundingSphere compute_enclosing_sphere_around_points(const std::vector<Eigen::Vector3d> &points_eigen,
																  const double padding = 0.0);

	std::vector<geometry_msgs::msg::Point> extract_leaf_vertices(const AppleTreePlanningScene &scene_info);

	std::vector<geometry_msgs::msg::Point> extract_leaf_vertices(const SimplifiedOrchard &orchard);
}

#endif //NEW_PLANNERS_ENCLOSING_SPHERE_H
