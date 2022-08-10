#include "math_utils.h"
#include <Eigen/Geometry>
#include <iostream>

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
	return (point - line.origin()).dot(line.direction()) / line.direction().squaredNorm();
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

Eigen::Vector3d closest_point_on_triangle(const Eigen::Vector3d &p,
										  const Eigen::Vector3d &va,
										  const Eigen::Vector3d &vb,
										  const Eigen::Vector3d &vc) {

	Eigen::Vector3d barycentric = project_barycentric(p, va, vb, vc);

	Eigen::Vector3d closest_point;
	double closest_distance = std::numeric_limits<double>::max();

	if (0 <= barycentric[0] && barycentric[0] <= 1 && 0 <= barycentric[1] && barycentric[1] <= 1 && 0 <= barycentric[2] && barycentric[2] <= 1) {
		closest_point = va * barycentric[0] + vb * barycentric[1] + vc * barycentric[2];
	} else {

		Eigen::ParametrizedLine<double, 3> ab(va, vb - va);
		Eigen::ParametrizedLine<double, 3> bc(vb, vc - vb);
		Eigen::ParametrizedLine<double, 3> ca(vc, va - vc);

		double t_ab = projectionParameter(ab, p);
		double t_bc = projectionParameter(bc, p);
		double t_ca = projectionParameter(ca, p);

		t_ab = std::clamp(t_ab, 0.0, 1.0);
		t_bc = std::clamp(t_bc, 0.0, 1.0);
		t_ca = std::clamp(t_ca, 0.0, 1.0);

		Eigen::Vector3d pt_ab = ab.pointAt(t_ab);
		Eigen::Vector3d pt_bc = bc.pointAt(t_bc);
		Eigen::Vector3d pt_ca = ca.pointAt(t_ca);

		double d_ab = (pt_ab - p).squaredNorm();
		double d_bc = (pt_bc - p).squaredNorm();
		double d_ca = (pt_ca - p).squaredNorm();

		if (d_ab < closest_distance) {
			closest_distance = d_ab;
			closest_point = pt_ab;
		}

		if (d_bc < closest_distance) {
			closest_distance = d_bc;
			closest_point = pt_bc;
		}

		if (d_ca < closest_distance) {
			closest_distance = d_ca;
			closest_point = pt_ca;
		}

	}

	return closest_point;

}

Eigen::Vector3d cheat_away_from_vertices(const Eigen::Vector3d &p,
										 const Eigen::Vector3d &va,
										 const Eigen::Vector3d &vb,
										 const Eigen::Vector3d &vc,
										 const double margin) {

	if ((p - va).squaredNorm() < margin*margin) {
		return va + ((vb + vc) / 2.0 - va).normalized() * margin;
	} else if ((p - vb).squaredNorm() < margin*margin) {
		return vb + ((vc + va) / 2.0 - vb).normalized() * margin;
	} else if ((p - vc).squaredNorm() < margin*margin) {
		return vc + ((va + vb) / 2.0 - vc).normalized() * margin;
	} else {
		return p;
	}

}

Plane3d plane_from_points(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3) {

	const Eigen::Vector3d normal = (p1 - p2).cross(p3 - p2).normalized();

	return {normal, -p1.dot(normal)};

}

