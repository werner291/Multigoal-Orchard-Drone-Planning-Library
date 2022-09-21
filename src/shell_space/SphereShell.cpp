#include "SphereShell.h"

#include <range/v3/all.hpp>

Eigen::Vector3d WorkspaceSphereShell::arm_vector(const Eigen::Vector3d &p) const {
	// Inwards-facing normal vector
	return (center - p).normalized();
}

Eigen::Vector3d WorkspaceSphereShell::nearest_point_on_shell(const Eigen::Vector3d &p) const {
	// Perpendicular projection onto the sphere
	return (p - center).normalized() * radius + center;
}

Eigen::Vector3d WorkspaceSphereShell::surface_point(const Eigen::Vector3d &p) const {
	// Check if the point is on the surface.
	assert(std::abs((p - center).norm() - radius) < 1e-6);

	// Just return it, this is an identity function.
	return p;
}

WorkspaceSphereShell::WorkspaceSphereShell(const Eigen::Vector3d &center, double radius)
		: center(center), radius(radius) {
}

std::shared_ptr<ShellPath<Eigen::Vector3d>>
WorkspaceSphereShell::path_from_to(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const {

	// Create a great circle path between the two points.
	return std::make_shared<GreatCirclePath>(from, to, center, radius);

}

Eigen::Vector3d WorkspaceSphereShell::random_near(const Eigen::Vector3d &p, double radius) const {

	// Get a random point on the sphere
	std::vector<double> pt(3);
	ompl::RNG rng;
	rng.uniformInBall(radius, pt);

	// Project it onto the sphere
	return nearest_point_on_shell(p + Eigen::Vector3d(pt[0], pt[1], pt[2]));
}

Eigen::Vector3d GreatCirclePath::at(double t) const {

	// Perform an angle-axis rotation of the start vector.
	return Eigen::AngleAxisd(t * angle, axis) * start_vec;

}

GreatCirclePath::GreatCirclePath(const Eigen::Vector3d &from,
								 const Eigen::Vector3d &to,
								 const Eigen::Vector3d &center,
								 double radius) :
		center(center),
		angle(std::acos((from-center).normalized().dot((to - center).normalized()))),
		start_vec(from - center),
		axis((from - center).cross(to - center).normalized()) {

	// Check that the points are on the sphere.
	assert(std::abs((from - center).norm() - radius) < 1e-6);
	assert(std::abs((to - center).norm() - radius) < 1e-6);

}

double GreatCirclePath::length() {
	return angle * start_vec.norm();
}
