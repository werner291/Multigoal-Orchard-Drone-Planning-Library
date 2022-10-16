
#include "TreeMeshes.h"

TreeMeshes loadTreeMeshes(const std::string &treeName) {
	TreeMeshes meshes;

	meshes.leaves_mesh = loadMesh(treeName + "_leaves.dae");
	meshes.trunk_mesh = loadMesh(treeName + "_trunk.dae");
	meshes.fruit_mesh = loadMesh(treeName + "_fruit.dae");

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
