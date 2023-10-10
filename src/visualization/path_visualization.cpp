
#include "path_visualization.h"

#include "../shell_space/path_util.h"

template <typename ShellPoint>
std::vector<Eigen::Vector3d> idealizedPathViaShell(const WorkspaceShell<ShellPoint> &shell,
												   const Eigen::Vector3d &start,
												   const Eigen::Vector3d &goal,
												   int curve_steps)
{

	auto shell_pt1 = shell.nearest_point_on_shell(start);
	auto shell_pt2 = shell.nearest_point_on_shell(goal);

	auto path = shell.path_from_to(shell_pt1, shell_pt2);

	auto path_points = shellPathToPoints(path, shell, curve_steps);

	// Add start and goal

	path_points.insert(path_points.begin(), start);
	path_points.push_back(goal);

	return path_points;
}

#include "../shell_space/CGALMeshShell.h"
#include "../shell_space/CuttingPlaneConvexHullShell.h"
#include "../shell_space/SphereShell.h"

// Explicit template instantiation
template std::vector<Eigen::Vector3d> idealizedPathViaShell(const WorkspaceShell<Eigen::Vector3d> &shell,
															const Eigen::Vector3d &start,
															const Eigen::Vector3d &goal,
															int curve_steps);

template std::vector<Eigen::Vector3d> idealizedPathViaShell(const WorkspaceShell<ConvexHullPoint> &shell,
															const Eigen::Vector3d &start,
															const Eigen::Vector3d &goal,
															int curve_steps);

template std::vector<Eigen::Vector3d> idealizedPathViaShell(const WorkspaceShell<mgodpl::cgal_utils::CGALMeshPointAndNormal> &shell,
															const Eigen::Vector3d &start,
															const Eigen::Vector3d &goal,
															int curve_steps);