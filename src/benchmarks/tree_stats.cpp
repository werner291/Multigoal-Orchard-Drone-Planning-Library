// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 8/6/24.
//

#include <iostream>
#include "benchmark_function_macros.h"

#include "../experiment_utils/TreeMeshes.h"

using namespace mgodpl::tree_meshes;

REGISTER_BENCHMARK(tree_stats) {
	// Grab a list of all tree models:
	const auto tree_model_names = getTreeModelNames();

	// Simply count the number of fruit models:
	for (const auto &tree_model_name: tree_model_names) {
		const auto &tree_model = loadTreeMeshes(tree_model_name);

		results[tree_model_name]["n_fruit"] = (int) tree_model.fruit_meshes.size();
		results[tree_model_name]["trunk_triangles"] = (int) tree_model.trunk_mesh.triangles.size();
		results[tree_model_name]["leaf_triangles"] = (int) tree_model.leaves_mesh.triangles.size();

		std::cout << "Tree model " << tree_model_name << " has " << tree_model.fruit_meshes.size() << " fruit models, "
				<< tree_model.trunk_mesh.triangles.size() << " trunk triangles, and "
				<< tree_model.leaves_mesh.triangles.size() << " leaf triangles." << std::endl;
	}
}
