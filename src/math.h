//
// Created by werner on 3-8-22.
//

#ifndef NEW_PLANNERS_MATH_H
#define NEW_PLANNERS_MATH_H

#include <utility>
#include <Eigen/src/Geometry/ParametrizedLine.h>

/**
 * Given two lines, expressed in origin-vector form, find the points of closest approach.
 *
 * @param l1 	The first line.
 * @param l2 	The second line.
 * @return 		Parameters t and s, such that l1.pointAt(t) and l2.pointAt(s) are the points of closest approach.
 */
std::pair<double, double> closest_point_on_line(
		const Eigen::ParametrizedLine<double, 3>& l1,
		const Eigen::ParametrizedLine<double, 3>& l2);

#endif //NEW_PLANNERS_MATH_H
