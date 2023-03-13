#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include "../TreeMeshes.h"
#include "../utilities/alpha_shape.h"
#include "../utilities/MeshOcclusionModel.h"

int main() {

	// Load the apple tree meshes.
	TreeMeshes meshes = loadTreeMeshes("appletree");

	auto alphashape = alphaShape(meshes.leaves_mesh.vertices | ranges::views::transform([](const auto &v) {
		return Eigen::Vector3d{v.x, v.y, v.z};
	}) | ranges::to_vector, LEAVES_ALPHA_SQRTRADIUS);

	MeshOcclusionModel occlusion_model(alphashape);



}