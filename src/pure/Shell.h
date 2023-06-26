// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 13-6-23.
//

#ifndef MGODPL_SHELL_H
#define MGODPL_SHELL_H

#include <vector>

namespace mgodpl {

	/**
	 * A template struct that represents the type of a shell point in the given shell space.
	 *
	 * A "Shell point" is a reduced, symbolic representation of a configuration in a shell space,
	 * abstracting away the actual configuration values and joint settings.
	 *
	 * @tparam Shell  The type of the shell space.
	 */
	template<typename Shell>
	struct shell_point_t {
		using type = typename Shell::ShellPoint;
	};

	/**
	 * A template struct that represents the type of a shell path in the given shell space.
	 *
	 * @tparam Shell  The type of the shell space.
	 */
	template<typename Shell>
	struct shell_path_t {
		using type = typename Shell::ShellPath;
	};

	/**
	 * Computes the path between two shell points within a shell space.
	 *
	 * @tparam Shell  The type of the shell space.
	 * @param shell        A reference to the shell space.
	 * @param sp1          The starting shell point.
	 * @param sp2          The ending shell point.
	 * @return             The path between the two shell points.
	 */
	template<typename Shell>
	typename shell_path_t<Shell>::type shell_path(const Shell& shell, const typename shell_point_t<Shell>::type& sp1, const typename shell_point_t<Shell>::type& sp2);

	/**
	 * Converts a shell point to a point in 3D space.
	 */
	template<typename Shell, typename Point>
	Point shell_point_to_euclidean(const Shell& shell, const typename shell_point_t<Shell>::type& sp);

	/**
	 * Calculates the normal vector at a shell point.
	 */
	template<typename Shell, typename Vector>
	Vector normal_at_shell_point(const Shell& shell, const typename shell_point_t<Shell>::type& sp);

	/**
	 * Calculates both the point and the normal vector at a shell point.
	 */
	template<typename Shell, typename Point, typename Vector>
	std::pair<Point, Vector> point_and_normal_at_shell_point(const Shell& shell, const typename shell_point_t<Shell>::type& sp) {
		return std::make_pair(shell_point_to_euclidean<Shell, Point>(shell, sp), normal_at_shell_point<Shell, Vector>(shell, sp));
	}

	/**
	 * Projects a point in 3D space to a shell point.
	 */
	template<typename Shell, typename Point>
	typename shell_point_t<Shell>::type project_euclidean_to_shell(const Shell& shell, const Point& p);

}


#endif //MGODPL_SHELL_H
