
#include "CylinderShell.h"

Eigen::Vector3d CylinderShell::arm_vector(const CylinderShellPoint &p) const {
	return {
		cos(p.angle), sin(p.angle), 0
	};
}

CylinderShellPoint CylinderShell::nearest_point_on_shell(const Eigen::Vector3d &p) const {
	return {
		atan2(p.y(), p.x()), p.z()
	};
}

Eigen::Vector3d CylinderShell::surface_point(const CylinderShellPoint &p) const {
	return {
		cos(p.angle) * radius, sin(p.angle) * radius, p.height
	};
}

std::shared_ptr<ShellPath<CylinderShellPoint>>
CylinderShell::path_from_to(const CylinderShellPoint &from, const CylinderShellPoint &to) const {
	return std::make_shared<HelixPath>(from, to, radius);
}

double CylinderShell::path_length(const std::shared_ptr<ShellPath<CylinderShellPoint>> &path) const {

	double angle_arc = std::dynamic_pointer_cast<HelixPath>(path)->delta_angle *
					   std::dynamic_pointer_cast<HelixPath>(path)->radius;
	double height_delta = std::dynamic_pointer_cast<HelixPath>(path)->delta_height;

	return std::sqrt(angle_arc * angle_arc + height_delta * height_delta);

}

CylinderShell::CylinderShell(double radius, const Eigen::Vector2d &center) : radius(radius), center(center) {
}

CylinderShellPoint CylinderShell::random_near(const CylinderShellPoint &p, double radius) const {

	thread_local std::random_device rd;
	thread_local std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(-0.5, 0.5);

	double angle = p.angle + dis(gen) * radius;
	double height = p.height + dis(gen) * radius;

	return {angle, height};

}

CylinderShellPoint HelixPath::at(double t) const {
	return {start.angle + t * delta_angle, start.height + t * delta_height};
}

double HelixPath::length() {
	return delta_angle * radius + delta_height;
}

std::shared_ptr<CylinderShell>
paddedCylindricalShellAroundLeaves(const AppleTreePlanningScene &scene_info, double padding) {

	// Find the mininum enclosing sphere.
	auto mec = utilities::compute_enclosing_sphere_around_leaves(*scene_info.scene_msg, padding);

	return std::make_shared<CylinderShell>(mec.radius, Eigen::Vector2d(mec.center.x(), mec.center.y()));

}
