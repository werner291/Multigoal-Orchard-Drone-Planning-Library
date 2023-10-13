// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 12-4-23.
//

#ifndef NEW_PLANNERS_SHELL_VISUALIZATION_H
#define NEW_PLANNERS_SHELL_VISUALIZATION_H

#include <vtkActor.h>
#include <vtkNew.h>
#include <vtkSmartPointer.h>

#include <functional>
#include <Eigen/Core>

struct Apple;
class WorkspaceSphereShell;

/**
 * Creates a vtkActor for a workspace sphere shell. The resulting actor is colored in light gray with
 * an opacity of 0.5.
 *
 * @param sphereshell The WorkspaceSphereShell to create the actor for.
 *
 * @return A vtkSmartPointer to a vtkActor representing the WorkspaceSphereShell.
 */
vtkSmartPointer<vtkActor> mkSphereShellActor(const WorkspaceSphereShell &sphereshell);

/// A function that computes a path between two points in the workspace.
using EuclideanSegmentedPathFn = std::function<std::vector<Eigen::Vector3d>(const Eigen::Vector3d &,
																			const Eigen::Vector3d &)>;

// A Rust trait-object-like wrapper around the various WorkspaceShell types
struct ShellWrapper {
	EuclideanSegmentedPathFn idealized_path;
	std::string name;
};

/**
 * Computes the idealized path edges between one apple and all other apples.
 *
 * @param apples The vector of apples.
 * @param apple_i The index of the apple to compute the path from.
 * @param path_fn The function to generate the path between two points.
 *
 * @return A vector of pairs of Eigen::Vector3d representing the edges between the specified apple and all other apples.
 */
std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
computeOneToAllIdealizedPathEdges(const std::vector<Apple> &apples,
								  int apple_i,
								  const EuclideanSegmentedPathFn &path_fn);

#endif //NEW_PLANNERS_SHELL_VISUALIZATION_H
