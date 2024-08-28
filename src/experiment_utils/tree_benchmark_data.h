// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-8-24.
//

#ifndef MGODPL_TREE_BENCHMARK_DATA_H
#define MGODPL_TREE_BENCHMARK_DATA_H

#include <vector>
#include <string>
#include <json/value.h>
#include "TreeMeshes.h"
#include "../planning/fcl_forward_declarations.h"
#include "../planning/cgal_chull_shortest_paths.h"

namespace mgodpl::experiments {

	/**
	 * @brief This function retrieves the tree model names, outputs them to the console,
	 * 		  and annotates the `results` JSON object with the tree model names.
	 *
	 * @param results A mutable reference to a Json::Value object that will be annotated with the tree model names.
	 * @return A std::vector of strings containing the names of the tree models.
	 */
	std::vector<std::string> getAndAnnotateTreeModels(Json::Value &results);

	/**
	 * @brief This struct holds data related to a tree model.
	 *
	 * It contains the tree model name, tree meshes, collision objects, and convex hull.
	 */
	struct TreeModelBenchmarkData {
		std::string tree_model_name;
		mgodpl::tree_meshes::TreeMeshes tree_mesh;
		std::shared_ptr<fcl::CollisionObjectd> tree_collision_object;
		std::shared_ptr<mgodpl::cgal::CgalMeshData> tree_convex_hull;
	};

	/**
	 * @brief This function retrieves data related to a tree model.
	 *
	 * It loads the tree meshes, creates a collision object, and creates a convex hull for the tree model.
	 *
	 * @param tree_model_name The name of the tree model.
	 * @return A TreeModelBenchmarkData object containing data related to the tree model.
	 */
	TreeModelBenchmarkData loadBenchmarkTreemodelData(const std::string &tree_model_name);

	/**
	 * @brief This function loads and prepares the benchmark data for all tree models, annotating the results with the tree model names.
	 *
	 * This function also prints to the console to inform the user about progress.
	 *
	 * @param results A mutable reference to a Json::Value object that will be annotated with the tree model names.
	 * @return A std::vector of TreeModelBenchmarkData objects containing the benchmark data for each tree model.
	 */
	std::vector<TreeModelBenchmarkData> loadAllTreeBenchmarkData(Json::Value &results);
}

#endif //MGODPL_TREE_BENCHMARK_DATA_H
