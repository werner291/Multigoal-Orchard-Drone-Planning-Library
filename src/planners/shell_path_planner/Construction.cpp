
#include "Construction.h"

std::shared_ptr <WorkspaceShell<Eigen::Vector3d>>
paddedSphericalShellAroundLeaves(const AppleTreePlanningScene &scene_info, double padding) {

	// Compute the minimum enclosing sphere of the leaves.
	auto enclosing = compute_enclosing_sphere(scene_info.scene_msg, 0.0);

	// Add padding to the radius.
	enclosing.radius += padding;

	// Construct a SphereShell with the computed center and radius.
	return std::make_shared<WorkspaceSphereShell>(enclosing.center, enclosing.radius);

}
