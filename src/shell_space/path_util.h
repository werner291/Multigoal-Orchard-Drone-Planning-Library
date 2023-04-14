// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 14-4-23.
//

#ifndef NEW_PLANNERS_PATH_UTIL_H
#define NEW_PLANNERS_PATH_UTIL_H

#include <vector>
#include <Eigen/Core>
#include <memory>
#include "WorkspaceShell.h"

/**
 * Converts a ShellPath to a list of points on the workspace shell surface.
 *
 * @param path The ShellPath to convert to points.
 * @param shell The WorkspaceShell on which the ShellPath is defined.
 * @param curve_steps The number of steps to use when discretizing a CurvePath. Ignored for PiecewiseLinearPath.
 *
 * @return A vector of Eigen::Vector3d representing points on the workspace shell surface.
 */
template<typename ShellPoint>
std::vector<Eigen::Vector3d> shellPathToPoints(const std::shared_ptr<ShellPath<ShellPoint>> &path,
											   const WorkspaceShell<ShellPoint> &shell,
											   int curve_steps);

#endif //NEW_PLANNERS_PATH_UTIL_H
