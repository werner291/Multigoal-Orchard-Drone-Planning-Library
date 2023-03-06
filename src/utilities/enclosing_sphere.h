//
// Created by werner on 6-3-23.
//

#ifndef NEW_PLANNERS_ENCLOSING_SPHERE_H
#define NEW_PLANNERS_ENCLOSING_SPHERE_H

#include <geometric_shapes/bodies.h>
#include "../planning_scene_diff_message.h"

namespace utilities {
	bodies::BoundingSphere
	compute_enclosing_sphere_around_leaves(const moveit_msgs::msg::PlanningScene &planning_scene_message,
										   const double padding);

	std::vector<geometry_msgs::msg::Point> extract_leaf_vertices(const AppleTreePlanningScene &scene_info);
}

#endif //NEW_PLANNERS_ENCLOSING_SPHERE_H
