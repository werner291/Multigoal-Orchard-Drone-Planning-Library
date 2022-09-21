
#ifndef NEW_PLANNERS_CYLINDERSHELL_H
#define NEW_PLANNERS_CYLINDERSHELL_H

#include "WorkspaceShell.h"

/**
 * A point on the cylinder.
 */
struct CylinderShellPoint {
	/// Angle around the vertical axis
	double angle;
	/// Height above z = 0
	double height;
};

/**
 * A helical path on the cylinder.
 */
class HelixPath : public CurvePath<CylinderShellPoint> {

	/// Start point
	CylinderShellPoint start;

	/// Change in angle
	double delta_angle;

	/// Change in height
	double delta_height;

	/// Radius of the cylinder (for length calculation)
	double radius;

public:

	/**
	 * Create a helical path between two points on the cylinder.
	 *
	 * @param from 			Start point
	 * @param to 			End point
	 */
	HelixPath(const CylinderShellPoint &from, const CylinderShellPoint &to, double radius) : radius(radius) {
		start = from;
		delta_angle = to.angle - from.angle;

		if (delta_angle > M_PI) {
			delta_angle -= 2 * M_PI;
		} else if (delta_angle < -M_PI) {
			delta_angle += 2 * M_PI;
		}

		delta_height = to.height - from.height;
	}

	[[nodiscard]] CylinderShellPoint at(double t) const override;

	double length() override;
};

/**
 * A cylindrical shell in workspace space.
 *
 * Shell paths are helixes on the cylinder, arm vectors are inwards-pointing normals at each point of the cylinder.
 */
class CylinderShell : public WorkspaceShell<CylinderShellPoint> {

	double radius;

public:
	explicit CylinderShell(double radius);

	[[nodiscard]] Eigen::Vector3d arm_vector(const CylinderShellPoint &p) const override;

	[[nodiscard]] CylinderShellPoint nearest_point_on_shell(const Eigen::Vector3d &p) const override;

	[[nodiscard]] Eigen::Vector3d surface_point(const CylinderShellPoint &p) const override;

	std::shared_ptr<ShellPath<CylinderShellPoint>>
	path_from_to(const CylinderShellPoint &from, const CylinderShellPoint &to) const override;

};


#endif //NEW_PLANNERS_CYLINDERSHELL_H
