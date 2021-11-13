//
// Created by werner on 18-08-21.
//

#ifndef NEW_PLANNERS_PLANNING_SCENE_DIFF_MESSAGE_H
#define NEW_PLANNERS_PLANNING_SCENE_DIFF_MESSAGE_H

#include "procedural_tree_generation.h"
#include <moveit/planning_scene/planning_scene.h>

moveit_msgs::PlanningScene createPlanningSceneDiff(std::vector<DetachedTreeNode> &treeFlattened,
                                                   std::vector<Eigen::Vector3d> &leafVertices,
                                                   double appleRadius,
                                                   std::vector<Apple> &apples);

#endif //NEW_PLANNERS_PLANNING_SCENE_DIFF_MESSAGE_H
