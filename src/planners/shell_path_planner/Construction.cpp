
#include "Construction.h"

std::shared_ptr<OmplShellSpace<ConvexHullPoint>>
cuttingPlaneChullShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) {
	auto workspaceShell = horizontalAdapter<ConvexHullPoint>(cuttingPlaneConvexHullAroundLeaves(scene_info, 0.0, 1.0));
	return OmplShellSpace<ConvexHullPoint>::fromWorkspaceShell(workspaceShell, si);
}

std::shared_ptr<OmplShellSpace<CGALMeshShellPoint>>
cgalPlaneChullShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) {
	auto workspaceShell = horizontalAdapter<CGALMeshShellPoint>(convexHullAroundLeavesCGAL(scene_info, 1.0, 0.0));
	return OmplShellSpace<CGALMeshShellPoint>::fromWorkspaceShell(workspaceShell, si);
}

std::shared_ptr<OmplShellSpace<CylinderShellPoint>>
cylinderShell(const AppleTreePlanningScene &scene, const ompl::base::SpaceInformationPtr &si) {
	return OmplShellSpace<CylinderShellPoint>::fromWorkspaceShell(horizontalAdapter<CylinderShellPoint>(
			paddedCylindricalShellAroundLeaves(scene, 0.0)), si);
}
