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
	 * Calculates the distance between two shell points within a shell space.
	 *
	 * @tparam Shell  The type of the shell space.
	 * @param shell        A reference to the shell space.
	 * @param sp1          The first shell point.
	 * @param sp2          The second shell point.
	 * @return             The distance between the two shell points.
	 */
	template<typename Shell>
	double shell_distance(const Shell& shell, const typename shell_point_t<Shell>::type& sp1, const typename shell_point_t<Shell>::type& sp2);

	/**
	 * Calculates the distances from a single shell point to all other shell points in a list within a shell space.
	 *
	 * @tparam Shell      The type of the shell space.
	 * @param shell            A reference to the shell space.
	 * @param shell_points     A vector of shell points.
	 * @return                 A vector of distances from the first shell point to all other shell points.
	 */
	template<typename Shell>
	std::vector<double> shell_distance_one_to_all(const Shell& shell, const std::vector<typename shell_point_t<Shell>::type>& shell_points) {
		std::vector<double> result;
		result.reserve(shell_points.size());

		for (const typename shell_point_t<Shell>::type& sp : shell_points) {
			result.push_back(shell_distance(shell, sp, shell_points[0]));
		}

		return result;
	}

	/**
	 * Calculates the distances between all pairs of shell points in a list within a shell space.
	 *
	 * @tparam Shell      The type of the shell space.
	 * @param shell            A reference to the shell space.
	 * @param shell_points     A vector of shell points.
	 * @return                 A two-dimensional vector representing the distances between all pairs of shell points.
	 */
	template<typename Shell>
	std::vector<std::vector<double>> shell_distance_all_to_all(const Shell& shell, const std::vector<typename shell_point_t<Shell>::type>& shell_points) {
		std::vector<std::vector<double>> result;
		result.reserve(shell_points.size());

		for (const typename shell_point_t<Shell>::type& sp1 : shell_points) {
			result.push_back(shell_distance_one_to_all(shell, shell_points));
		}

		return result;
	}

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
