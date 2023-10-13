// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 14-5-23.
//

#include <random>
#include "orchards.h"
#include "utilities/mesh_utils.h"

std::vector<TreeAtPosition> createOrchardRow(const std::vector<TreeMeshes> &treeModels,
											 double interval,
											 int num_samples) {

	// Initialize a random number generator
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> dis(0, treeModels.size() - 1);
	std::uniform_real_distribution<> angle_dis(0, 2 * M_PI); // Generate random angle in range [0, 2 * pi]

	// Sample elements from treeModels with replacement
	std::vector<TreeAtPosition> sampledTreeModels;
	Eigen::Vector3d step(interval, 0.0, 0.0);

	for (int i = 0; i < num_samples; ++i) {
		int random_index = dis(gen);
		TreeAtPosition sampledTree;
		sampledTree.meshes = treeModels[random_index];
		sampledTree.position = i * step; // Set the position based on the fixed interval
		sampledTree.rotation_angle = angle_dis(gen); // Set a random rotation angle about the z-axis
		sampledTreeModels.push_back(sampledTree);
	}

	return sampledTreeModels;
}

TreeMeshes mergeTreeMeshes(const std::vector<TreeAtPosition> &trees) {
	// Merge the meshes
	TreeMeshes treeMeshes;
	treeMeshes.tree_name = "Orchard";
	for (auto tree : trees) {
		append_mesh(treeMeshes.trunk_mesh, translate_mesh(tree.meshes.trunk_mesh, tree.position));
		append_mesh(treeMeshes.leaves_mesh, translate_mesh(tree.meshes.leaves_mesh, tree.position));
		for (const auto& fruit : tree.meshes.fruit_meshes) {
			treeMeshes.fruit_meshes.push_back(translate_mesh(fruit, tree.position));
		}
	}
	return treeMeshes;
}
