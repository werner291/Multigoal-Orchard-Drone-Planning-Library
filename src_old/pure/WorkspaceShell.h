// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

/**
 * @file workspace_shell_adapter.h
 *
 * This file contains template specializations and functions that adapt the
 * WorkspaceShell class to work with the traits defined in mgodpl namespace.
 *
 * The shell traits are defined in terms of types and functions that can be
 * used to interact with shells and points in shells. The WorkspaceShell class
 * is adapted to work with these traits by providing implementations for various
 * template functions.
 *
 * The template functions in this file use the functionality provided by the
 * WorkspaceShell class to perform operations such as calculating distances,
 * computing paths, converting between shell points and Euclidean points, and
 * calculating normal vectors at shell points.
 *
 * The implementation leverages dynamic dispatch of the virtual methods of
 * WorkspaceShell, providing a level of abstraction and enabling the use of
 * different WorkspaceShell implementations.
 *
 * However, thanks to the template functions, the dynamic dispatch is only
 * an implementation detail and the user of the functions does not need to
 * be aware of it.
 */

#ifndef MGODPL_WORKSPACESHELL_H
#define MGODPL_WORKSPACESHELL_H

#include "../shell_space/WorkspaceShell.h"

#include "Shell.h"

namespace mgodpl {

	template<typename ShellPoint, typename ShellPath> using WorkspaceShell = ::WorkspaceShell<ShellPoint, ShellPath>;

	template<typename ShellPoint, typename ShellPath>
	struct shell_point_t<WorkspaceShell<ShellPoint, ShellPath>> {
		using type = ShellPoint;
	};

	template<typename ShellPoint, typename ShellPath>
	struct shell_path_t<WorkspaceShell<ShellPoint, ShellPath>> {
		using type = ShellPath;
	};

	template<typename ShellPoint, typename ShellPath>
	double shell_distance(const WorkspaceShell<ShellPoint, ShellPath>& shell, const ShellPoint& sp1, const ShellPoint& sp2) {
		return shell.path_length(shell.path_from_to(sp1, sp2));
	}

	template<typename ShellPoint, typename ShellPath>
	std::vector<std::vector<double>> shell_distance_all_to_all(const WorkspaceShell<ShellPoint, ShellPath>& shell,
															   const std::vector<ShellPoint>& shell_points) {
		return shell.distance_matrix(shell_points);
	}

	template<typename ShellPoint, typename ShellPath>
	typename shell_path_t<WorkspaceShell<ShellPoint, ShellPath>>::type
	shell_path(const WorkspaceShell<ShellPoint, ShellPath>& shell, const ShellPoint& sp1, const ShellPoint& sp2) {
		return shell.path_from_to(sp1, sp2);
	}

	template<typename ShellPoint, typename ShellPath>
	Eigen::Vector3d shell_point_to_euclidean(const WorkspaceShell<ShellPoint, ShellPath>& shell, const ShellPoint& sp) {
		return shell.surface_point(sp);
	}

	template<typename ShellPoint, typename ShellPath>
	Eigen::Vector3d normal_at_shell_point(const WorkspaceShell<ShellPoint, ShellPath>& shell, const ShellPoint& sp) {
		return shell.arm_vector(sp);
	}

	template<typename ShellPoint, typename ShellPath>
	std::pair<Eigen::Vector3d, Eigen::Vector3d> point_and_normal_at_shell_point(const WorkspaceShell<ShellPoint, ShellPath>& shell, const ShellPoint& sp) {
		return std::make_pair(shell.surface_point(sp), shell.arm_vector(sp));
	}

	template<typename ShellPoint, typename ShellPath>
	ShellPoint project_euclidean_to_shell(const WorkspaceShell<ShellPoint, ShellPath>& shell, const Eigen::Vector3d& p) {
		return shell.nearest_point_on_shell(p);
	}
}


#endif //MGODPL_WORKSPACESHELL_H
