/**
 * @file TreeMeshes.h
 * @brief Defines the TreeMeshes structure and related functions for handling and processing tree meshes.
 *
 * This header defines a structure to store tree meshes, separated by trunk, leaves, and fruit (fruit further broken down into individual fruit).
 * It also includes several functions to load the meshes, get tree model names, create an initial state of a drone, and get tree names.
 *
 * The code assumes that the tree models exist in a specific directory and have a specific naming convention (i.e., "treename_trunk.dae", "treename_leaves.dae", "treename_fruit.dae").
 *
 * @note For fruit meshes, some meshes that are too small are considered to be "stem" of the fruit and are removed.
 *       The code also assumes that the fruit meshes can be broken down into connected components.
 *
 * @author Werner Kroneman
 * @date 15-05-2023
 *
 * Copyright 2023 University College Roosevelt
 * All Rights Reserved
 */

#ifndef NEW_PLANNERS_TREEMESHES_H
#define NEW_PLANNERS_TREEMESHES_H

#include <shape_msgs/msg/mesh.hpp>
#include <Eigen/Core>
#include <moveit/robot_state/robot_state.h>

/**
 * @struct TreeMeshes
 * @brief A structure to store the meshes of a tree.
 *
 * This structure contains the name of the tree and separate meshes for leaves, trunk, and fruit.
 * The fruit meshes are stored in a vector, with each fruit being a separate mesh.
 */
struct TreeMeshes {
	std::string tree_name;
	shape_msgs::msg::Mesh leaves_mesh;
	shape_msgs::msg::Mesh trunk_mesh;
	std::vector<shape_msgs::msg::Mesh> fruit_meshes;
};

/**
 * @brief Loads the meshes for the given tree.
 *
 * This function loads the meshes for the leaves, trunk, and fruit of the given tree from the specific directory.
 * The fruit meshes are further broken down into individual fruit.
 *
 * @param treeName The name of the tree to load the meshes for.
 * @return The loaded meshes.
 */
TreeMeshes loadTreeMeshes(const std::string &treeName);

/**
 * @brief Fetches the names of all tree models present in the specified directory.
 *
 * This function iterates through all the files in the directory and checks for files with extension ".dae" and stem "_trunk".
 * It extracts the name of the tree from these file names and returns a vector of all such names.
 *
 * @param path The path of the directory to search. Default is "./3d-models/".
 * @return A vector of tree model names.
 */
std::vector<std::string> getTreeModelNames(const std::string path = "./3d-models/");

/**
 * @struct SimplifiedOrchard
 * @brief A structure to store trees in an orchard.
 *
 * This structure stores a vector of pairs. Each pair consists of a 2D Eigen vector and a TreeMeshes object,
 * and represents a tree in the orchard. The 2D Eigen vector represents the position of the root of the tree
 * on the XY-plane floor/ground of the orchard environment.
 */
struct SimplifiedOrchard {
	std::vector<std::pair<Eigen::Vector2d, TreeMeshes>> trees;
};

/**
 * @brief Creates an initial state for a drone.
 *
 * This function generates a RobotState object for a drone with a single predetermined joint configuration.
 *
 * @param drone Pointer to the RobotModel of the drone.
 * @return The initial state of the drone.
 */
[[deprecated("Always generates the same state.")]]
moveit::core::RobotState mkInitialState(const moveit::core::RobotModelPtr &drone);

/**
 * Get a list of tree model names.
 *
 * @return 		A vector of tree model names.
 */
[[deprecated("Use getTreeModelNames() instead.")]]
std::vector<std::string> getTreeNames();

#endif //NEW_PLANNERS_TREEMESHES_H
