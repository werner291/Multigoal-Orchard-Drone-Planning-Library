#ifndef NEW_PLANNERS_CYLINDERSHELL_H
#define NEW_PLANNERS_CYLINDERSHELL_H

#include "WorkspaceShell.h"
#include "../AppleTreePlanningScene.h"
#include "../utilities/enclosing_sphere.h"

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


public:
	double length();

	/// Start point
	CylinderShellPoint start{};

	/// Change in angle
	double delta_angle;

	/// Change in height
	double delta_height;

	/// Radius of the cylinder (for length calculation)
	double radius;

	/**
	 * Create a helical path between two points on the cylinder.
	 *
	 * @param from 			Start point
	 * @param to 			End point
	 */
	HelixPath(const CylinderShellPoint &from, const CylinderShellPoint &to, double radius);

	[[nodiscard]] CylinderShellPoint at(double t) const override;
};

/**
 * A cylindrical shell in workspace space.
 *
 * Shell paths are helixes on the cylinder, arm vectors are inwards-pointing normals at each point of the cylinder.
 */
class CylinderShell : public WorkspaceShell<CylinderShellPoint> {
	
public:

	double radius;
	Eigen::Vector2d center;

	/**
	 * Constructor of the CylinderShell class.
	 *
	 * @param radius Radius of the cylindrical shell.
	 * @param center Center of the cylindrical shell in the XY plane.
	 */
	CylinderShell(double radius, const Eigen::Vector2d &center);

public:

	/**
	 * Computes the arm vector at the given point on the cylindrical shell.
	 *
	 * @param p A point on the cylindrical shell.
	 * @return The inwards-pointing normal vector at the given point.
	 */
	[[nodiscard]] Eigen::Vector3d arm_vector(const CylinderShellPoint &p) const override;

	/**
	 * Computes the nearest point on the cylindrical shell to a given point in 3D space.
	 *
	 * @param p A point in 3D space.
	 * @return The nearest point on the cylindrical shell.
	 */
	[[nodiscard]] CylinderShellPoint nearest_point_on_shell(const Eigen::Vector3d &p) const override;

	/**
	 * Computes the 3D coordinates of a given point on the cylindrical shell.
	 *
	 * @param p A point on the cylindrical shell.
	 * @return The 3D coordinates of the given point.
	 */
	[[nodiscard]] Eigen::Vector3d surface_point(const CylinderShellPoint &p) const override;

	/**
	 * Computes a path between two points on the cylindrical shell.
	 *
	 * @param from The start point on the cylindrical shell.
	 * @param to The end point on the cylindrical shell.
	 * @return A shared_ptr to a HelixPath representing the path between the two points.
	 */
	[[nodiscard]] std::shared_ptr<ShellPath<CylinderShellPoint>>
	path_from_to(const CylinderShellPoint &from, const CylinderShellPoint &to) const override;

	/**
	 * Generates a random point on the cylindrical shell within a specified radius of a given point.
	 *
	 * @param p A point on the cylindrical shell.
	 * @param distribution_radius The radius within which to generate the random point.
	 * @return A randomly generated point on the cylindrical shell within the specified radius.
	 */
	[[nodiscard]] CylinderShellPoint random_near(const CylinderShellPoint &p, double distribution_radius) const override;

	/**
	 * Computes the length of a given path on the cylindrical shell.
	 *
	 * @param path A shared_ptr to a ShellPath representing a path on the cylindrical shell.
	 * @return The length of the path.
	 */
	double path_length(const std::shared_ptr<ShellPath<CylinderShellPoint>> &path) const override;
};


std::shared_ptr<CylinderShell>
paddedCylindricalShellAroundLeaves(const AppleTreePlanningScene &scene_info, double padding);

#endif //NEW_PLANNERS_CYLINDERSHELL_H
