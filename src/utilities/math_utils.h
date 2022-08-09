//
// Created by werner on 3-8-22.
//

#ifndef NEW_PLANNERS_MATH_UTILS_H
#define NEW_PLANNERS_MATH_UTILS_H

#include <utility>
#include <Eigen/Geometry>

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

/**
 * Given a line and a point, find the projection of the point onto the line, expressed as a parameter for the line.
 *
 * @param line 		The line.
 * @param point 	The point.
 * @return 			The parameter t, such that line.pointAt(t) is the projection of the point onto the line.
 */
double projectionParameter(const Eigen::ParametrizedLine<double, 3>& line, const Eigen::Vector3d& point);

/**
 *
 * Compute the barycentric coordinates of a point in the plane defined by the three vertices of a triangle.
 *
 * Based on StackExchange answer here: https://math.stackexchange.com/a/2579920
 *
 * The function returns a vector (alpha,beta,gamma) such that qp' == alpha*va + beta*vb + gamma*vc where
 * qp' is the projection of the query point into the plane defined by the three vertices of the triangle.
 *
 * @param qp	The point to compute the barycentric coordinates of.
 * @param va	The first vertex of the triangle.
 * @param vb	The second vertex of the triangle.
 * @param vc	The third vertex of the triangle.
 * @return		The barycentric coordinates of the point (alpha, beta, gamma).
 */
Eigen::Vector3d project_barycentric(const Eigen::Vector3d &qp,
									const Eigen::Vector3d &va,
									const Eigen::Vector3d &vb,
									const Eigen::Vector3d &vc);


Eigen::Vector3d closest_point_on_triangle(const Eigen::Vector3d &p,
										  const Eigen::Vector3d &va,
										  const Eigen::Vector3d &vb,
										  const Eigen::Vector3d &vc);

#endif //NEW_PLANNERS_MATH_UTILS_H
