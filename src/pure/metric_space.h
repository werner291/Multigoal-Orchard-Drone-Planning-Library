/**
 * @file metric_space.h
 * @brief Provides functions for calculating distances between points within a metric space.
 *
 * This file contains template functions for calculating distances between points within a given metric space.
 * A metric space is a set where a notion of distance (called a metric) is defined between every pair of points.
 * It provides functions to calculate the distance between two points, distances from a single point
 * to all other points in a list, and distances between all pairs of points in a list.
 *
 * We treat the notion of a "Space" explicitly, and consider the possibility of such a space
 * having a non-trivial internal structure that requires a reference to that space to be passed
 * to the distance calculation functions.
 *
 * This is by contrast to simple Euclidean spaces, where the distance between two points is
 * simply the Euclidean distance between them, and the distance calculation functions do not
 * require a reference to the space to be passed to them.
 *
 * @copyright (c) 2022 University College Roosevelt
 * All rights reserved.
 */

#ifndef MGODPL_METRIC_SPACE_H
#define MGODPL_METRIC_SPACE_H

#include <vector>

#include "topological_space.h"

namespace mgodpl::metric_space {


	/**
	 * Calculates the distance between two points within a given metric space.
	 *
	 * @tparam Space  	The type of the metric space.
	 * @param space   	A reference to the metric space.
	 * @param p1      	The first point.
	 * @param p2      	The second point.
	 * @return        	The distance between the two points.
	 */
	template<typename Space>
	double point_distance(const Space& context, const typename space_point_t<Space>::type& p1, const typename space_point_t<Space>::type& p2);

	/**
	 * Calculates the distances from a single point to all other points in a list within a given context.
	 *
	 * @tparam Space    The type of the context.
	 *
	 * @param context     A reference to the context, or space in which the points are defined.
	 * @param p1		  The first point.
	 * @param points      A vector of points.
	 * @return            A vector of distances from the first point to all other points.
	 */
	template<typename Space>
	std::vector<double> point_distance_one_to_all(const Space& context,
												  const typename space_point_t<Space>::type& p1,
												  const std::vector<typename space_point_t<Space>::type>& points) {
		std::vector<double> result;
		result.reserve(points.size());

		for (const typename space_point_t<Space>::type& p : points) {
			result.push_back(point_distance(context, p, p1));
		}

		return result;
	}

	/**
	 * Calculates the distances between all pairs of points in a list within a given context.
	 *
	 * @tparam Space    The type of the context.
	 * @param context     A reference to the context.
	 * @param points      A vector of points.
	 * @return            A two-dimensional vector representing the distances between all pairs of points.
	 */
	template<typename Space>
	std::vector<std::vector<double>> point_distance_all_to_all(const Space& context, const std::vector<typename space_point_t<Space>::type>& points) {
		std::vector<std::vector<double>> result;
		result.reserve(points.size());

		for (const typename space_point_t<Space>::type& p1 : points) {
			result.push_back(point_distance_one_to_all(context, p1, points));
		}

		return result;
	}

	/**
	 * Given a path in the space, calculates the length of the path.
	 */
	template<typename Space>
	double path_length(const Space& context, const typename space_path_t<Space>::type& path);

}

#endif //MGODPL_METRIC_SPACE_H
