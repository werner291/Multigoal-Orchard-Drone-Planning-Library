#include <filesystem>
#include "TreeMeshes.h"
#include "WorkspaceSpec.h"
#include "procedural_tree_generation.h"
#include "utilities/mesh_utils.h"
#include "utilities/load_mesh.h"

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
	meshes.fruit_meshes.erase(std::remove_if(meshes.fruit_meshes.begin(), meshes.fruit_meshes.end(), [](const auto &mesh) {
		return mesh.vertices.size() <= 5;
	}), meshes.fruit_meshes.end());

	size_t n_after = meshes.fruit_meshes.size();

	if (n_before != n_after) {
		std::cout << "Removed " << n_before - n_after << " out of " << n_before << " fruit meshes that were too small from model " << treeName << std::endl;
	}

	return meshes;
}

moveit::core::RobotState mkInitialState(const moveit::core::RobotModelPtr &drone) {
	moveit::core::RobotState current_state(drone);
	current_state.setVariablePositions({5.0, 0.0, 1.5, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0});
	current_state.update();
	return current_state;
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

