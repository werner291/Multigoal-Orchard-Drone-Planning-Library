// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_TREEMESHES_H
#define NEW_PLANNERS_TREEMESHES_H

#include <shape_msgs/msg/mesh.hpp>
#include "../math/Vec3.h"

namespace mgodpl::tree_meshes {

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
		std::vector<std::pair<math::Vec3d, TreeMeshes>> trees;
	};

	/**
	 * Get a list of tree model names.
	 *
	 * @return 		A vector of tree model names.
	 */
	[[deprecated("Use getTreeModelNames() instead.")]]
	std::vector<std::string> getTreeNames();

	/**
	 * @brief Load randomly selected tree models with a limit on the number of fruit meshes.
	 *
	 * This function loads tree models randomly until it reaches the specified number (n). It only considers
	 * tree models with a number of fruit meshes less than or equal to max_fruit. Tree models are loaded
	 * in the order provided by a random shuffle of the tree model names.
	 *
	 * We limit fruit numbers because some tree models are very large and crash the program.
	 *
	 * @param n The number of tree models to load.
	 * @param max_fruit The maximum number of fruit meshes a tree model can have to be considered.
	 * @return A vector of loaded tree models.
	 */
	std::vector<TreeMeshes> loadRandomTreeModels(int n, int max_fruit);

	/**
	 * @brief Load the tree models with a limit on the number of fruit meshes.
	 *
	 * We limit fruit numbers because some tree models are very large and crash the program.
	 *
	 * @param max_n The number of tree models to load.
	 * @param max_fruit The maximum number of fruit meshes a tree model can have to be considered.
	 * @return A vector of loaded tree models.
	 */
	std::vector<TreeMeshes> loadAllTreeModels(int max_n, int max_fruit);

	/**
	 * @brief Create a single row orchard from a given vector of tree models.
	 *
	 * This function creates a simplified representation of an orchard where all the trees are placed
	 * in a single row, with a constant displacement between each tree. The tree models are placed
	 * in the order they appear in the input vector.
	 *
	 * @param tree_models A vector of tree models to place in the orchard.
	 * @return A SimplifiedOrchard object representing the created orchard.
	 */
	SimplifiedOrchard makeSingleRowOrchard(std::vector<TreeMeshes> &tree_models);

}

#endif //NEW_PLANNERS_TREEMESHES_H
