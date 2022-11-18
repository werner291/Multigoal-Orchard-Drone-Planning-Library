
#include "TreeMeshes.h"
#include "WorkspaceSpec.h"
#include "procedural_tree_generation.h"
#include "utilities/mesh_utils.h"

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

	meshes.leaves_mesh = loadMesh(treeName + "_leaves.dae");
	meshes.trunk_mesh = loadMesh(treeName + "_trunk.dae");

	auto fruit_meshes = loadMesh(treeName + "_fruit.dae");

	meshes.fruit_meshes = break_down_to_connected_components(fruit_meshes);

	return meshes;
}

moveit::core::RobotState mkInitialState(const moveit::core::RobotModelPtr &drone) {
	moveit::core::RobotState current_state(drone);
	current_state.setVariablePositions({
											   5.0, 0.0, 1.5,
											   0.0, 0.0, 0.0, 1.0,
											   0.0, 0.0, 0.0, 0.0
									   });
	current_state.update();
	return current_state;
}

