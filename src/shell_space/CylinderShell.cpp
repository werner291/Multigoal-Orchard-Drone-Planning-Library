
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
	return std::dynamic_pointer_cast<HelixPath>(path)->delta_angle * std::dynamic_pointer_cast<HelixPath>(path)->radius
	       + std::dynamic_pointer_cast<HelixPath>(path)->delta_height;
}

CylinderShellPoint HelixPath::at(double t) const {
	return {
		start.angle + t * delta_angle,
		start.height + t * delta_height
	};
}

double HelixPath::length() {
	return delta_angle * radius + delta_height;
}
