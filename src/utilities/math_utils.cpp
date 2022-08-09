#include "math_utils.h"
#include <Eigen/Geometry>

std::pair<double, double>
closest_point_on_line(const Eigen::ParametrizedLine<double, 3> &l1,
					  const Eigen::ParametrizedLine<double, 3> &l2) {

	// Direction vectors of both lines.
	const Eigen::Vector3d& d1 = l1.direction();
	const Eigen::Vector3d& d2 = l2.direction();

	// Formula from	https://en.wikipedia.org/wiki/Skew_lines#Nearest_points

	// The edge connecting the two points of closest approach must be perpendicular to both vectors.
	// Hence, we can use the cross product to find a direction vector of the edge. (but not the magnitude)
	Eigen::Vector3d n = d1.cross(d2);

	// Compute the normal of the plane containing d1 perpendicular to n.
	Eigen::Vector3d n1 = d1.cross(n);

	// Compute the normal of the plane containing d2 perpendicular to n.
	Eigen::Vector3d n2 = d2.cross(n);

	return {
		n2.dot(l2.origin() - l1.origin()) / n2.dot(d1),
		n1.dot(l1.origin() - l2.origin()) / n1.dot(d2)
	};

}

double projectionParameter(const Eigen::ParametrizedLine<double, 3> &line, const Eigen::Vector3d &point) {
	return (point - line.origin()).dot(line.direction());
}

Eigen::Vector3d project_barycentric(const Eigen::Vector3d &qp,
									const Eigen::Vector3d &va,
									const Eigen::Vector3d &vb,
									const Eigen::Vector3d &vc) {
	// u=P2−P1
	Eigen::Vector3d u = vb - va;
	// v=P3−P1
	Eigen::Vector3d v = vc - va;
	// n=u×v
	Eigen::Vector3d n = u.cross(v);
	// w=P−P1
	Eigen::Vector3d w = qp - va;
	// Barycentric coordinates of the projection P′of P onto T:
	// γ=[(u×w)⋅n]/n²
	double gamma = u.cross(w).dot(n) / n.dot(n);
	// β=[(w×v)⋅n]/n²
	double beta = w.cross(v).dot(n) / n.dot(n);

	// Must sum to 1.
	double alpha = 1 - gamma - beta;

	return {alpha, beta, gamma};
}
