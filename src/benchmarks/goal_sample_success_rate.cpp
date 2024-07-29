// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 7/26/24.
//

#include <iostream>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include <range/v3/view/transform.hpp>
#include <range/v3/to_container.hpp>

#include "benchmark_function_macros.h"
#include "../experiment_utils/LoadedTreeModel.h"

#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/tree_models.h"
#include "../planning/fcl_utils.h"

REGISTER_BENCHMARK(goal_sample_success_rate) {
	std::cout << "Running goal_sample_success_rate" << std::endl;

	// Grab a list of all tree models:
	const auto tree_model_names = mgodpl::tree_meshes::getTreeModelNames();

	std::cout << "Will evaluate for the following tree models:";
	for (const auto &tree_model: tree_model_names) {
		std::cout << " " << tree_model;
	}
	std::cout << std::endl;

	// Note the tree names:
	results["tree_models"] = Json::arrayValue;
	for (const auto &tree_model: tree_model_names) {
		results["tree_models"].append(tree_model);
	}

	// Load the tree meshes for each tree:
	std::vector<mgodpl::tree_meshes::TreeMeshes> tree_meshes;
	for (const auto &tree_model_name: tree_model_names) {
		tree_meshes.push_back(mgodpl::tree_meshes::loadTreeMeshes(tree_model_name));
	}

	// Make CollisionObjectd's for each tree:
	std::vector<fcl::CollisionObjectd> tree_collision_objects;
	for (const auto &tree_mesh: tree_meshes) {
		tree_collision_objects.emplace_back(mgodpl::fcl_utils::meshToFclBVH(tree_mesh.trunk_mesh));
	}
}
