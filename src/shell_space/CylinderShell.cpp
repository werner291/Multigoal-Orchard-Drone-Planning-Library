

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
		cos(p.angle), sin(p.angle), p.height
	};
}
