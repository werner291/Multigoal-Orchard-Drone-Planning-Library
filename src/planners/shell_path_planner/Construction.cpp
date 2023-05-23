
#include "Construction.h"

std::shared_ptr<OmplShellSpace<ConvexHullPoint>>
cuttingPlaneChullShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) {
	auto workspaceShell = horizontalAdapter<ConvexHullPoint>(cuttingPlaneConvexHullAroundLeaves(scene_info, 0.0, 1.0));
	return OmplShellSpace<ConvexHullPoint>::fromWorkspaceShell(workspaceShell, si);
}

std::shared_ptr<OmplShellSpace<CGALMeshShellPoint>>
cgalChullShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) {
	auto workspaceShell = horizontalAdapter<CGALMeshShellPoint>(convexHullAroundLeavesCGAL(scene_info, 1.0, 0.0));
	return OmplShellSpace<CGALMeshShellPoint>::fromWorkspaceShell(workspaceShell, si);
}

std::shared_ptr<OmplShellSpace<CylinderShellPoint>>
cylinderShell(const AppleTreePlanningScene &scene, const ompl::base::SpaceInformationPtr &si) {
	return OmplShellSpace<CylinderShellPoint>::fromWorkspaceShell(horizontalAdapter<CylinderShellPoint>(
			paddedCylindricalShellAroundLeaves(scene, 0.0)), si);
}

/**
 * @brief Constructs an OmplShellSpace with a cutting plane convex hull around the leaves of the apple tree.
 * This function conforms to the MkOmplShellFn template.
 *
 * @param scene_info The AppleTreePlanningScene containing information about the apple tree.
 * @param si The ompl::base::SpaceInformationPtr for the constructed OmplShellSpace.
 * @return A shared_ptr to the OmplShellSpace containing the cutting plane convex hull.
 */
std::shared_ptr<OmplShellSpace<Eigen::Vector3d>>
minimumEnclosingSphereShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) {
	auto workspaceShell = horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene_info, 0.0));
	return OmplShellSpace<Eigen::Vector3d>::fromWorkspaceShell(workspaceShell, si);
}