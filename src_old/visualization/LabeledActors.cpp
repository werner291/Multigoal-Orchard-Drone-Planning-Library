
#include "LabeledActors.h"

#include "../utilities/vtk.h"
#include "../utilities/mesh_utils.h"
#include "../TreeMeshes.h"

LabeledActors treeMeshesToLabeledActors(const TreeMeshes &treeMeshes) {
	return {{{createColoredMeshActor(treeMeshes.trunk_mesh, {0.5, 0.3, 0.1, 1.0})}, "trunk"},
			{{createColoredMeshActor(treeMeshes.leaves_mesh, {0.1, 0.5, 0.1, 1.0})}, "leaves"},
			{{createColoredMeshActor(createGroundPlane(10.0, 10.0), {0.5, 0.5, 0.1, 1.0})}, "ground"},
			{{createColoredMeshActors(treeMeshes.fruit_meshes, {1.0, 0.0, 0.0, 1.0})}, "apples"}};
}