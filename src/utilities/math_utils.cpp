#include "math_utils.h"
#include <Eigen/Geometry>
#include <iostream>

#include <random_numbers/random_numbers.h>
#include <ompl/util/RandomNumbers.h>

std::pair<double, double>
closest_point_on_line(const Eigen::ParametrizedLine<double, 3> &l1,
					  const Eigen::ParametrizedLine<double, 3> &l2) {

	// Direction vectors of both lines.
	const Eigen::Vector3d& d1 = l1.direction();
	const Eigen::Vector3d& d2 = l2.direction();

	// Formula from	https://en.wikipedia.org/wiki/Skew_lines#Nearest_points

	// The edge connecting the two points of on_which_mesh approach must be perpendicular to both vectors.
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

std::array<TriangleEdgeId, 2> edges_adjacent_to_vertex(TriangleVertexId vertex) {

	switch (vertex) {
		case VERTEX_A:
			return {EDGE_AB, EDGE_CA};
		case VERTEX_B:
			return {EDGE_BC, EDGE_AB};
		case VERTEX_C:
			return {EDGE_CA, EDGE_BC};
		default:
			throw std::runtime_error("Invalid vertex");
	}

}

std::array<TriangleVertexId, 2> vertices_in_edge(TriangleEdgeId edge) {
	switch (edge) {
		case EDGE_AB:
			return {VERTEX_A, VERTEX_B};
		case EDGE_BC:
			return {VERTEX_B, VERTEX_C};
		case EDGE_CA:
			return {VERTEX_C, VERTEX_A};
		default:
			throw std::runtime_error("Invalid edge id");
	}
}

Eigen::Vector3d
uniform_point_on_triangle(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3) {

	ompl::RNG rng;

	// Pick a point on a triangle
	const double r1 = sqrt(rng.uniform01());
	const double r2 = sqrt(rng.uniform01());

	return (1 - r1) * p1 + r1 * (1 - r2) * p2 + r1 * r2 * p3;
}

bool hollow_sphere_intersects_hollow_aabb(const Eigen::Vector3d &sphere_center,
										  const double sphere_radius,
										  const Eigen::AlignedBox3d &aabb) {

	// Check if the boundary of the sphere is within the bounds of the node.
	// Algorithm from https://stackoverflow.com/a/41457896 (hollow sphere case)

	double dmin = 0; // Distance from the sphere center to the closest point on the AABB
	double dmax = 0; // Distance from the sphere center to the farthest point on the AABB
	bool face = false;

	double square_radius = std::pow(sphere_radius, 2);

	for (int dim = 0; dim < 3; dim++) {
		// Square distances between the sphere center and the extended planes of the AABB in this dimension
		double a = std::pow(sphere_center[dim] - aabb.min()[dim], 2);
		double b = std::pow(sphere_center[dim] - aabb.max()[dim], 2);

		// std::max(a, b) would be the greatest squared distance from the sphere center to the extended planes of the AABB in this dimension
		// So, we add this squared distance to the total, building up one component of the squared distance from the sphere center to the AABB
		// in every iteration. ( x^2 + y^2 + z^2, where each term gets added in a different iteration )
		dmax += std::max(a, b);


		if (sphere_center[dim] < aabb.min()[dim]) {
			face = true;
			dmin += a;
		} else if (sphere_center[dim] > aabb.max()[dim]) {
			face = true;
			dmin += b;
		} else if (std::min(a, b) <= square_radius)
			face = true;
	}

	return (face && (dmin <= square_radius) && (square_radius <= dmax));
}

OctantIterator::OctantIterator(size_t i, const Eigen::AlignedBox3d &bounds) : i(i), bounds(bounds) {
}

OctantIterator::OctantIterator(const Eigen::AlignedBox3d &bounds) : i(0), bounds(bounds) {
}

OctantIterator OctantIterator::end() {
	return {8, Eigen::AlignedBox3d()};
}

OctantIterator &OctantIterator::operator++() {
	++i;
	return *this;
}

bool OctantIterator::operator==(const OctantIterator &other) const {
	return i == other.i && bounds.min() == other.bounds.min() && bounds.max() == other.bounds.max();
}

bool OctantIterator::operator!=(const OctantIterator &other) const {
	return !(*this == other);
}

Eigen::AlignedBox3d OctantIterator::operator*() const {

	Eigen::AlignedBox3d result;

	Eigen::Vector3d center = bounds.center();

	result.min().x() = (i & 1) ? bounds.min().x() : center.x();
	result.min().y() = (i & 2) ? bounds.min().y() : center.y();
	result.min().z() = (i & 4) ? bounds.min().z() : center.z();

	result.max().x() = (i & 1) ? center.x() : bounds.max().x();
	result.max().y() = (i & 2) ? center.y() : bounds.max().y();
	result.max().z() = (i & 4) ? center.z() : bounds.max().z();

	return result;

}

OctantIterator OctantIterator::operator++(int) {

	OctantIterator result = *this;
	++(*this);
	return result;

}
