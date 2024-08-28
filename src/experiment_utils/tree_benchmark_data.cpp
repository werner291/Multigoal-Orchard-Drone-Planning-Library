// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-8-24.
//

#include <iostream>
#include "tree_benchmark_data.h"
#include "../planning/fcl_utils.h"

namespace mgodpl::experiments {
	std::vector<std::string> getAndAnnotateTreeModels(Json::Value &results) {
		// Grab a list of all tree models:
		auto tree_model_names = mgodpl::tree_meshes::getTreeModelNames();

		// Print the names of the tree models that will be evaluated:
		std::cout << "Will evaluate for the following tree models:";
		for (const auto &tree_model: tree_model_names) {
			std::cout << " " << tree_model;
		}
		std::cout << std::endl;
		// Print the total number of tree models:
		std::cout << "Total of " << tree_model_names.size() << " tree models." << std::endl;

		// Annotate the `results` JSON object with the tree model names:
		results["tree_models"] = Json::arrayValue;
		for (const auto &tree_model: tree_model_names) {
			results["tree_models"].append(tree_model);
		}

		// Return the list of tree model names:
		return tree_model_names;
	}

	TreeModelBenchmarkData
	loadBenchmarkTreemodelData(const std::string &tree_model_name) {
		std::string local_tree_model_name = tree_model_name;
		auto local_tree_mesh = mgodpl::tree_meshes::loadTreeMeshes(tree_model_name);
		std::cout << "Creating collision object for tree model " << tree_model_name << std::endl;
		auto local_tree_collision_object = std::make_shared<fcl::CollisionObjectd>(mgodpl::fcl_utils::meshToFclBVH(
				local_tree_mesh.trunk_mesh));
		std::cout << "Creating convex hull for tree model " << tree_model_name << std::endl;
		auto local_tree_convex_hull = std::make_shared<mgodpl::cgal::CgalMeshData>(local_tree_mesh.leaves_mesh);

		return {
				local_tree_model_name,
				local_tree_mesh,
				local_tree_collision_object,
				local_tree_convex_hull
		};
	}

	std::vector<TreeModelBenchmarkData> loadAllTreeBenchmarkData(Json::Value &results) {
		// Get the tree model names and annotate the results
		auto tree_model_names = getAndAnnotateTreeModels(results);

		// Vector to hold the tree benchmark data
		std::vector<TreeModelBenchmarkData> all_tree_data;

		// Load the benchmark data for each tree model
		for (const auto &tree_model_name: tree_model_names) {
			all_tree_data.push_back(loadBenchmarkTreemodelData(tree_model_name));
		}

		return all_tree_data;
	}
}