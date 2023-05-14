
#include "CylinderShell.h"

Eigen::Vector3d CylinderShell::arm_vector(const CylinderShellPoint &p) const {
	// Calculate the inwards-pointing normal vector at the given point
	return {
			-cos(p.angle), -sin(p.angle), 0
	};
}

CylinderShellPoint CylinderShell::nearest_point_on_shell(const Eigen::Vector3d &p) const {
	// Compute the nearest point on the cylindrical shell to the given point in 3D space
	return {
			atan2(p.y(), p.x()), p.z()
	};
}

Eigen::Vector3d CylinderShell::surface_point(const CylinderShellPoint &p) const {
	// Calculate the 3D coordinates of a given point on the cylindrical shell
	return {
			cos(p.angle) * radius, sin(p.angle) * radius, p.height
	};
}

std::shared_ptr<ShellPath<CylinderShellPoint>>
CylinderShell::path_from_to(const CylinderShellPoint &from, const CylinderShellPoint &to) const {
	// Create a helix path between two points on the cylindrical shell
	return std::make_shared<HelixPath>(from, to, radius);
}

double CylinderShell::path_length(const std::shared_ptr<ShellPath<CylinderShellPoint>> &path) const {
	// Calculate the angle arc and height delta for the given helix path
	double angle_arc = std::dynamic_pointer_cast<HelixPath>(path)->delta_angle *
					   std::dynamic_pointer_cast<HelixPath>(path)->radius;
	double height_delta = std::dynamic_pointer_cast<HelixPath>(path)->delta_height;

	// Calculate the length of the helix path
	return std::sqrt(angle_arc * angle_arc + height_delta * height_delta);
}

CylinderShell::CylinderShell(double radius, const Eigen::Vector2d &center)
		: radius(radius), center(center) {
	// Constructor for the CylinderShell class
}

CylinderShellPoint CylinderShell::random_near(const CylinderShellPoint &p, double distribution_radius) const {
	// Initialize random number generator
	thread_local std::random_device rd;
	thread_local std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(-0.5, 0.5);

	// Generate random angle and height within the specified radius
	double angle = p.angle + dis(gen) * distribution_radius;
	double height = p.height + dis(gen) * distribution_radius;

	// Create a new CylinderShellPoint with the random angle and height
	return {angle, height};
}

CylinderShellPoint HelixPath::at(double t) const {
	// Calculate the point on the helix path at a given parameter t
	return {start.angle + t * delta_angle, start.height + t * delta_height};
}

double HelixPath::length() {
	// Calculate the length of the helix path
	return delta_angle * radius + delta_height;
}

HelixPath::HelixPath(const CylinderShellPoint &from, const CylinderShellPoint &to, double radius)
		: radius(radius) {
	start = from;
	delta_angle = to.angle - from.angle;

	// Normalize the angle difference to be between -M_PI and M_PI
	if (delta_angle > M_PI) {
		delta_angle -= 2 * M_PI;
	} else if (delta_angle < -M_PI) {
		delta_angle += 2 * M_PI;
	}

	delta_height = to.height - from.height;
}

std::shared_ptr<CylinderShell>
paddedCylindricalShellAroundLeaves(const AppleTreePlanningScene &scene_info, double padding) {

	// Find the mininum enclosing sphere.
	auto mec = utilities::compute_enclosing_sphere_around_leaves(*scene_info.scene_msg, padding);

	return std::make_shared<CylinderShell>(mec.radius, Eigen::Vector2d(mec.center.x(), mec.center.y()));

}
