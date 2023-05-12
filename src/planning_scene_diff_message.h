
#ifndef NEW_PLANNERS_PLANNING_SCENE_DIFF_MESSAGE_H
#define NEW_PLANNERS_PLANNING_SCENE_DIFF_MESSAGE_H

#include <moveit/planning_scene/planning_scene.h>

#include "procedural_tree_generation.h"
#include "TreeMeshes.h"

struct AppleTreePlanningScene {
	std::shared_ptr<moveit_msgs::msg::PlanningScene> scene_msg;
	std::vector<Apple> apples;
};

std::vector<AppleTreePlanningScene> scenes_for_trees(const std::vector<std::string> &tree_names, int max_fruit = 500);

const double TRANSLATION_BOUND = 10.0;

moveit_msgs::msg::PlanningScene
treeMeshesToMoveitSceneMsg(const TreeMeshes &tree_meshes, bool include_ground_plane = true);

AppleTreePlanningScene
createMeshBasedAppleTreePlanningSceneMessage(const std::string &model_name, bool include_ground_plane);


#endif //NEW_PLANNERS_PLANNING_SCENE_DIFF_MESSAGE_H
