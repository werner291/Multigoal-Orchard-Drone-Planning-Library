/**
 * @file distance_matrix.h
 * @brief Provides functions for calculating distances between points within a context.
 *
 * This file contains template functions for calculating distances between points within a given context.
 * It provides functions to calculate the distance between two points, distances from a single point
 * to all other points in a list, and distances between all pairs of points in a list.
 *
 * @copyright (c) 2022 University College Roosevelt
 * All rights reserved.
 */

#ifndef MGODPL_DISTANCE_MATRIX_H
#define MGODPL_DISTANCE_MATRIX_H

#include <vector>

namespace mgodpl::distance_matrix {

	/**
	 * Calculates the distance between two points within a given context.
	 *
	 * @tparam Context  The type of the context.
	 * @tparam Point    The type of the points.
	 * @param context   A reference to the context.
	 * @param p1        The first point.
	 * @param p2        The second point.
	 * @return          The distance between the two points.
	 */
	template<typename Context, typename Point>
	double point_distance(const Context& context, const Point& p1, const Point& p2);

	/**
	 * Calculates the distances from a single point to all other points in a list within a given context.
	 *
	 * @tparam Context    The type of the context.
	 * @tparam Point      The type of the points.
	 *
	 * @param context     A reference to the context, or space in which the points are defined.
	 * @param p1		  The first point.
	 * @param points      A vector of points.
	 * @return            A vector of distances from the first point to all other points.
	 */
	template<typename Context, typename Point>
	std::vector<double> point_distance_one_to_all(const Context& context,
												  const Point& p1,
												  const std::vector<Point>& points) {
		std::vector<double> result;
		result.reserve(points.size());

		for (const Point& p : points) {
			result.push_back(point_distance(context, p, p1));
		}

		return result;
	}

	/**
	 * Calculates the distances between all pairs of points in a list within a given context.
	 *
	 * @tparam Context    The type of the context.
	 * @tparam Point      The type of the points.
	 * @param context     A reference to the context.
	 * @param points      A vector of points.
	 * @return            A two-dimensional vector representing the distances between all pairs of points.
	 */
	template<typename Context, typename Point>
	std::vector<std::vector<double>> point_distance_all_to_all(const Context& context, const std::vector<Point>& points) {
		std::vector<std::vector<double>> result;
		result.reserve(points.size());

		for (const Point& p1 : points) {
			result.push_back(point_distance_one_to_all(context, points));
		}

		return result;
	}

}

#endif //MGODPL_DISTANCE_MATRIX_H
