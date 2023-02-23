#include "SphereShell.h"
#include "../utilities/experiment_utils.h"

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

double WorkspaceSphereShell::path_length(const std::shared_ptr<ShellPath<Eigen::Vector3d>> &path) const {
	auto great_circle_path = std::dynamic_pointer_cast<GreatCirclePath>(path);

	assert(great_circle_path != nullptr);

	// The length of the great circle path is the arc length of the great circle.
	return great_circle_path->length();
}

const Eigen::Vector3d &WorkspaceSphereShell::getCenter() const {
	return center;
}

double WorkspaceSphereShell::getRadius() const {
	return radius;
}

Eigen::Vector3d GreatCirclePath::at(double t) const {

	// Perform an angle-axis rotation of the start vector.
	return Eigen::AngleAxisd(t * angle, axis) * start_vec + center;

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

std::shared_ptr<WorkspaceSphereShell>
paddedSphericalShellAroundLeaves(const AppleTreePlanningScene &scene_info, double padding) {

	// Compute the minimum enclosing sphere of the leaves.
	auto enclosing = compute_enclosing_sphere_around_leaves(*scene_info.scene_msg, 0.0);

	// Add padding to the radius.
	enclosing.radius += padding;

	// Construct a SphereShell with the computed center and radius.
	return std::make_shared<WorkspaceSphereShell>(enclosing.center, enclosing.radius);

}

