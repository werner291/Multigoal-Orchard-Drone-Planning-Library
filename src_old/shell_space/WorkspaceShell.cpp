

#include "WorkspaceShell.h"
#include "CuttingPlaneConvexHullShell.h"
#include "CGALMeshShell.h"
#include "CylinderShell.h"

template<typename ShellPoint, typename ShellPath>
std::vector<std::vector<double>>
WorkspaceShell<ShellPoint, ShellPath>::distance_matrix(const std::vector<ShellPoint> &points) const {

	// By default, just call path_from_to for each pair of points and record the length of the path.

	std::vector<std::vector<double>> result(points.size(), std::vector<double>(points.size(), 0.0));

	for (size_t i = 0; i < points.size(); i++) {
		for (size_t j = 0; j < points.size(); j++) {
			result[i][j] = path_length(path_from_to(points[i], points[j]));
		}
	}

	return result;

}

template class WorkspaceShell<Eigen::Vector3d>;
template class WorkspaceShell<mgodpl::cgal_utils::CGALMeshPointAndNormal>;
template class WorkspaceShell<ConvexHullPoint>;
template class WorkspaceShell<CylinderShellPoint>;