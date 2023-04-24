
#pragma once

#include <vector>
#include <Eigen/Core>

template<typename ShellPoint>
class WorkspaceShell;

template <typename ShellPoint>
std::vector<Eigen::Vector3d> idealizedPathViaShell(const WorkspaceShell<ShellPoint> &shell,
												   const Eigen::Vector3d &start,
												   const Eigen::Vector3d &goal,
												   int curve_steps);