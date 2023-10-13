// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <filesystem>
#include <range/v3/action/shuffle.hpp>
#include <iostream>

#include "TreeMeshes.h"
#include "load_mesh.h"
#include "mesh_connected_components.h"

namespace mgodpl::tree_meshes {
	/**
	 * Loads the tree meshes for the tree with the given name.
	 *
	 * Models are assumed to exist in the following three paths:
	 *
	 *  - $(cwd)/3d-models/$(treeName)_trunk.dae
	 *  - $(cwd)/3d-models/$(treeName)_leaves.dae
	 *  - $(cwd)/3d-models/$(treeName)_fruit.dae
	 *
	 * These are assumed to be COLLADA .dae files. Individual fruit are assumed to be connected components of the _fruit mesh.
	 *
	 * @param treeName	The name of the tree to load.
	 * @return			The tree meshes, separated by trunk, leaves, and fruit (fruit further broken down into individual fruit).
	 */
	TreeMeshes loadTreeMeshes(const std::string &treeName) {
		TreeMeshes meshes;

		meshes.tree_name = treeName;

		meshes.leaves_mesh = loadMesh(treeName + "_leaves.dae");
		meshes.trunk_mesh = loadMesh(treeName + "_trunk.dae");

		auto fruit_meshes = loadMesh(treeName + "_fruit.dae");

		meshes.fruit_meshes = break_down_to_connected_components(fruit_meshes);

		size_t n_before = meshes.fruit_meshes.size();

		// Some meshes are actually tiny sliver triangles that represent the "Stem" of the fruit. WE should ignore these.
		meshes.fruit_meshes
				.erase(std::remove_if(meshes.fruit_meshes.begin(), meshes.fruit_meshes.end(), [](const auto &mesh) {
					return mesh.vertices.size() <= 5;
				}), meshes.fruit_meshes.end());

		size_t n_after = meshes.fruit_meshes.size();

		if (n_before != n_after) {
			std::cout << "Removed " << n_before - n_after << " out of " << n_before
					  << " fruit meshes that were too small from model " << treeName << std::endl;
		}

		return meshes;
	}

	std::vector<std::string> getTreeModelNames(const std::string path) {

		std::vector<std::string> modelNames;

		for (const auto &entry: std::filesystem::directory_iterator(path)) {

			// Check if the file is a *_trunk.dae file

			if (entry.path().extension() == ".dae" &&
				entry.path().stem().string().find("_trunk") != std::string::npos) {

				// Extract the name of the file excluding the _trunk.dae part and add it to the vector
				modelNames.push_back(entry.path()
											 .stem()
											 .string()
											 .substr(0, entry.path().stem().string().find("_trunk")));
			}
		}

		return modelNames;

	}

	bool endsWith(std::string_view str, std::string_view suffix) {
		return str.size() >= suffix.size() && 0 == str.compare(str.size() - suffix.size(), suffix.size(), suffix);
	}

	std::vector<std::string> getTreeNames() {

		// Check inside the 3d-models directory for all the tree names.
		// Tree models always come in 3 files: treename_leaves.dae, treename_trunk.dae, treename_fruit.dae
		// So we can just look for all the files ending in _fruit.dae and strip off the _fruit.dae to get the tree name.

		std::vector<std::string> tree_names;

		std::string tree_model_directory = "./3d-models/";

		const std::string tree_model_suffix = "_fruit.dae";

		for (const auto &entry: std::filesystem::directory_iterator(tree_model_directory)) {

			if (entry.is_regular_file()) {

				std::string filename = entry.path().filename().string();

				if (endsWith(filename, tree_model_suffix)) {

					std::string tree_name = filename.substr(0, filename.size() - tree_model_suffix.size());

					tree_names.push_back(tree_name);

				}

			}

		}

		return tree_names;

	}

	std::vector<TreeMeshes> loadRandomTreeModels(const int n, const int max_fruit) {

		// Initialize a random number generator.
		std::mt19937 gen(std::random_device{}());

		// Get the names of available tree models and shuffle them.
		auto tree_names = getTreeModelNames() | ranges::actions::shuffle(gen);

		// Create a vector to hold the loaded tree models.
		std::vector<TreeMeshes> tree_models;

		// Loop over the shuffled tree model names.
		for (const auto &name: tree_names) {
			// Load the tree model.
			auto models = loadTreeMeshes(name);

			// If the tree model has a suitable number of fruit meshes, add it to the vector.
			if (models.fruit_meshes.size() <= max_fruit) {
				tree_models.push_back(models);

				// If we've loaded enough tree models, stop.
				if (tree_models.size() == n) {
					break;
				}
			}
		}

		// Return the vector of loaded tree models.
		return tree_models;
	}

	std::vector<TreeMeshes> loadAllTreeModels(int max_n, int max_fruit) {

		// Get the names of available tree models and shuffle them.
		auto tree_names = getTreeModelNames();

		// Create a vector to hold the loaded tree models.
		std::vector<TreeMeshes> tree_models;

		// Loop over the shuffled tree model names.
		for (const auto &name: tree_names) {
			// Load the tree model.
			auto models = loadTreeMeshes(name);

			// If the tree model has a suitable number of fruit meshes, add it to the vector.
			if (models.fruit_meshes.size() <= max_fruit) {
				tree_models.push_back(models);

				// If we've loaded enough tree models, stop.
				if (tree_models.size() == max_n) {
					break;
				}
			}
		}

		// Return the vector of loaded tree models.
		return tree_models;

	}

	SimplifiedOrchard makeSingleRowOrchard(std::vector<TreeMeshes> &tree_models) {
		double x_displacement = tree_models.size() * 2.0 * -0.5;
		SimplifiedOrchard orchard;

		for (const auto &model: tree_models) {
			orchard.trees.push_back({{x_displacement, 0.0,0.0}, model});
			x_displacement += 2.0;
		}
		return orchard;
	}
}

