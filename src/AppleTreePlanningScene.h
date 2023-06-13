/**
 * @file AppleTreePlanningScene.h
 * @brief Defines data structures and functions related to the creation and management of AppleTreePlanningScene.
 *
 * This file contains the definition of the AppleTreePlanningScene structure and related functions.
 * These functions involve creating AppleTreePlanningScene objects from TreeMeshes, generating PlanningScene messages,
 * and managing the scenes for multiple trees.
 *
 * @warning Some operations in this file involve expensive convex decomposition operations that are cached to disk.
 *
 * @author Werner Kroneman
 * @date 2023-05-15
 *
 * Copyright 2023 University College Roosevelt
 * All rights reserved.
 */

#ifndef NEW_PLANNERS_APPLETREEPLANNINGSCENE_H
#define NEW_PLANNERS_APPLETREEPLANNINGSCENE_H

#include <moveit/planning_scene/planning_scene.h>

#include "procedural_tree_generation.h"
#include "TreeMeshes.h"

const double TRANSLATION_BOUND = 10.0;

/**
 * @brief The AppleTreePlanningScene structure holds a PlanningScene message and a vector of Apples.
 *
 * The PlanningScene message is used to represent the environment as a combination of a PlanningScene message
 * and a vector of Apples, which serve as targets for the robot to visit.
 *
 * @note The initial robot state is not included in this structure;
 * 		 the robot state field in the PlanningScene message is unused.
 */
struct AppleTreePlanningScene {
	std::shared_ptr<moveit_msgs::msg::PlanningScene> scene_msg; ///< The PlanningScene message associated with the apple tree.
	std::vector<Apple> apples; ///< The vector of apples associated with the apple tree.
};

/**
 * @brief Returns a vector of AppleTreePlanningScenes for a given set of tree names.
 *
 * This function creates an AppleTreePlanningScene for each tree name in the provided vector,
 * but skips trees that have more than the specified maximum number of fruit.
 *
 * @param tree_names The names of the trees for which to create AppleTreePlanningScenes.
 * @param max_fruit The maximum number of fruit allowed for a tree to be included. Default is 500.
 *
 * @return A vector of AppleTreePlanningScenes for the valid trees.
 */
std::vector<AppleTreePlanningScene> scenes_for_trees(const std::vector<std::string> &tree_names, const size_t max_fruit = 600);

/**
 * @brief Converts a TreeMeshes object into a PlanningScene message.
 *
 * This function generates a PlanningScene message from a given TreeMeshes object.
 * It includes an option to include a ground plane in the scene.
 *
 * @note Apples/fruit are not included in the generated PlanningScene
 * 		 message to avoid having them count as collision objects.
 *
 * @param tree_meshes The TreeMeshes object to convert.
 * @param include_ground_plane Whether to include a ground plane in the scene. Default is true.
 *
 * @return The generated PlanningScene message.
 */
moveit_msgs::msg::PlanningScene treeMeshesToMoveitSceneMsg(const TreeMeshes &tree_meshes, bool include_ground_plane = true);

/**
 * @brief Creates an AppleTreePlanningScene based on a mesh model.
 *
 * This function creates a PlanningScene message based on the specified model name (loading it from disk) and includes
 * an option to include a ground plane. It then uses the message to create an AppleTreePlanningScene.
 *
 * @warning This function involves an expensive convex decomposition operation that is cached to disk based on the model_name.
 *
 * @param model_name The name of the model to use.
 * @param include_ground_plane Whether to include a ground plane in the scene. Default is true.
 *
 * @return The created AppleTreePlanningScene.
 */
AppleTreePlanningScene
createMeshBasedAppleTreePlanningSceneMessage(const std::string &model_name, bool include_ground_plane);

/**
 * @brief Creates an AppleTreePlanningScene from a given TreeMeshes.
 *
 * This function processes the fruit meshes in the TreeMeshes to create a vector of Apple objects.
 * It then constructs an AppleTreePlanningScene with a PlanningScene message converted from the TreeMeshes and the vector of Apples.
 *
 * @warning This function involves an expensive convex decomposition operation that is cached to disk based on the tree_name in TreeMeshes.
 *
 * @param tree_models The TreeMeshes structure from which to create the AppleTreePlanningScene.
 *
 * @warning This function involves an expensive convex decomposition operation that is cached to disk based on the tree_name in TreeMeshes.
 *
 * @return The created AppleTreePlanningScene.
 */
AppleTreePlanningScene createSceneFromTreeModels(const TreeMeshes &tree_models);

/**
 * @brief Creates an apple tree planning scene from a simplified orchard.
 *
 * This function creates an apple tree planning scene using the tree meshes in the simplified orchard.
 * Each tree in the orchard is added to the scene.
 *
 * @warning This function involves an expensive convex decomposition operation for every tree in the orchard.
 * 			Trees are cached to disk based on the tree_name in each tree.
 *
 * @param orchard The simplified orchard to create the scene from.
 * @return The created apple tree planning scene.
 */
AppleTreePlanningScene createSceneFromSimplifiedOrchard(const SimplifiedOrchard& orchard);

#endif //NEW_PLANNERS_APPLETREEPLANNINGSCENE_H
