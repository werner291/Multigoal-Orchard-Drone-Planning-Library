
#ifndef NEW_PLANNERS_PLANNING_SCENE_DIFF_MESSAGE_H
#define NEW_PLANNERS_PLANNING_SCENE_DIFF_MESSAGE_H

#include <moveit/planning_scene/planning_scene.h>

#include "procedural_tree_generation.h"

void spawnApplesInPlanningScene(double appleRadius,
                                const std::vector<Apple> &apples,
                                moveit_msgs::msg::PlanningScene &planning_scene_diff);

moveit_msgs::msg::PlanningScene createPlanningSceneDiff(const std::vector<DetachedTreeNode> &treeFlattened,
                                                   const std::vector<Eigen::Vector3d> &leafVertices,
                                                   const double appleRadius,
                                                   const std::vector<Apple> &apples);

struct AppleTreePlanningScene {
    moveit_msgs::msg::PlanningScene scene_msg;
    std::vector<Apple> apples;
};

const std::initializer_list<size_t> DIFFICULT_APPLES {80, 79, 88, 76, 78, 3, 62, 21, 11, 16};

const double TRANSLATION_BOUND = 10.0;

AppleTreePlanningScene
createMeshBasedAppleTreePlanningSceneMessage(const std::string &model_name, bool include_ground_plane);



#endif //NEW_PLANNERS_PLANNING_SCENE_DIFF_MESSAGE_H
