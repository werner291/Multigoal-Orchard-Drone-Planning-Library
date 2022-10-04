//
// Created by werner on 4-10-22.
//

#include "TreeMeshes.h"

TreeMeshes loadTreeMeshes(const std::string &treeName) {
	TreeMeshes meshes;

	meshes.leaves_mesh = loadMesh(treeName + "_leaves.dae");
	meshes.trunk_mesh = loadMesh(treeName + "_trunk.dae");
	meshes.fruit_mesh = loadMesh(treeName + "_fruit.dae");

	return meshes;
}
