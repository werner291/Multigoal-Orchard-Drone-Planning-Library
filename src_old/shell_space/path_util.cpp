// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "path_util.h"
#include "CuttingPlaneConvexHullShell.h"
#include "CGALMeshShell.h"

template<typename ShellPoint>
std::vector<Eigen::Vector3d> shellPathToPoints(const std::shared_ptr<ShellPath<ShellPoint>> &path,
											   const WorkspaceShell<ShellPoint> &shell,
											   int curve_steps) {

	std::vector<Eigen::Vector3d> points;

	if (auto curve = std::dynamic_pointer_cast<CurvePath<ShellPoint>>(path)) {

		for (int i = 0; i < curve_steps; i++) {
			auto t = i / (double) curve_steps;
			auto p = curve->at(t);
			auto surface_point = shell.surface_point(p);
			points.push_back(surface_point);
		}

	} else if (auto piecewise = std::dynamic_pointer_cast<PiecewiseLinearPath<ShellPoint>>(path)) {

		for (auto &p: piecewise->points) {
			points.push_back(shell.surface_point(p));
		}

	}

	return points;

}

// Explicit instantiation for Vector3d, ConvexHullPoint, and CGALMeshSHellPoint

template std::vector<Eigen::Vector3d> shellPathToPoints(
		const std::shared_ptr<ShellPath<Eigen::Vector3d>> &path,
		const WorkspaceShell<Eigen::Vector3d> &shell,
		int curve_steps);

template std::vector<Eigen::Vector3d> shellPathToPoints(
		const std::shared_ptr<ShellPath<ConvexHullPoint>> &path,
		const WorkspaceShell<ConvexHullPoint> &shell,
		int curve_steps);

template std::vector<Eigen::Vector3d> shellPathToPoints(
		const std::shared_ptr<ShellPath<mgodpl::cgal_utils::CGALMeshPointAndNormal>> &path,
		const WorkspaceShell<mgodpl::cgal_utils::CGALMeshPointAndNormal> &shell,
		int curve_steps);


