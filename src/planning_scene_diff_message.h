
#ifndef NEW_PLANNERS_PLANNING_SCENE_DIFF_MESSAGE_H
#define NEW_PLANNERS_PLANNING_SCENE_DIFF_MESSAGE_H

#include <moveit/planning_scene/planning_scene.h>

#include "procedural_tree_generation.h"

void spawnApplesInPlanningScene(double appleRadius,
                                const std::vector<Apple> &apples,
                                moveit_msgs::PlanningScene &planning_scene_diff);

moveit_msgs::PlanningScene createPlanningSceneDiff(const std::vector<DetachedTreeNode> &treeFlattened,
                                                   const std::vector<Eigen::Vector3d> &leafVertices,
                                                   const double appleRadius,
                                                   const std::vector<Apple> &apples);

struct AppleTreePlanningScene {
    moveit_msgs::PlanningScene scene_msg;
    std::vector<Apple> apples;
};

AppleTreePlanningScene createMeshBasedAppleTreePlanningSceneMessage();

#endif //NEW_PLANNERS_PLANNING_SCENE_DIFF_MESSAGE_H
