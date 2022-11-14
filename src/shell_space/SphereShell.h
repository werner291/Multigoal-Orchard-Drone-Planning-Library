#ifndef NEW_PLANNERS_SPHERESHELL_H
#define NEW_PLANNERS_SPHERESHELL_H

#include <moveit/robot_state/robot_state.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/ScopedState.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <ompl/base/Goal.h>
#include "../planning_scene_diff_message.h"
#include "../utilities/moveit_conversions.h"
#include "../ompl_custom.h"
#include "WorkspaceShell.h"
#include "ArmHorizontal.h"

/**
 * A geodesic path on a sphere between two points.
 */
class GreatCirclePath : public CurvePath<Eigen::Vector3d> {

	/// Center of the sphere.
	const Eigen::Vector3d center;
	/// Vector perpendicular to the arc of rotation
	const Eigen::Vector3d axis;
	/// Total rotation angle
	const double angle;
	/// Start point relative to the center.
	const Eigen::Vector3d start_vec;

public:

	/**
	 * Create a great circle path between two points on a sphere.
	 *
	 * @param from 			Start point (in global space)
	 * @param to 			End point	(in global space)
	 * @param center 		Center of the sphere
	 * @param radius 		Radius of the sphere
	 */
	GreatCirclePath(const Eigen::Vector3d& from, const Eigen::Vector3d& to, const Eigen::Vector3d& center, double radius);

	/**
	 * Get the point on the sphere at a given fraction of the path.
	 *
	 * @param t 	Fraction of the path (0 <= t <= 1)
	 * @return 		Point on the sphere
	 */
	[[nodiscard]] Eigen::Vector3d at(double t) const override;

	double length();

};

/**
 * A spherical shell in workspace space.
 *
 * Shell paths are geodesic paths on the sphere, arm vectors are inwards-pointing normals at each point of the sphere.
 */
class WorkspaceSphereShell : public WorkspaceShell<Eigen::Vector3d> {

	/// Center of the sphere
	Eigen::Vector3d center;
public:
	const Eigen::Vector3d &getCenter() const;

	double getRadius() const;

private:

	/// Radius of the sphere
	double radius;

public:
	/**
	 * Create a spherical shell.
	 * @param center 		Center of the sphere
	 * @param radius 		Radius of the sphere
	 */
	WorkspaceSphereShell(const Eigen::Vector3d &center, double radius);

	/**
	 *
	 * Shell path between two points on the sphere.
	 *
	 * @param from 	Start point (in global space)
	 * @param to 	End point (in global space)
	 * @return 		Geodesic path between the two points
	 */
	[[nodiscard]] std::shared_ptr<ShellPath<Eigen::Vector3d>>
	path_from_to(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const override;

	[[nodiscard]] Eigen::Vector3d arm_vector(const Eigen::Vector3d &p) const override;

	[[nodiscard]] Eigen::Vector3d nearest_point_on_shell(const Eigen::Vector3d &p) const override;

	[[nodiscard]] Eigen::Vector3d surface_point(const Eigen::Vector3d &p) const override;

	double path_length(const std::shared_ptr<ShellPath<Eigen::Vector3d>> &path) const override;
};

/**
 * Constructs a WorkspaceShell based on the minimum enclosing sphere of the leaves in a scene.
 *
 * @param scene_info 		The scene information.
 * @param padding 			The padding to add to the radius of the sphere.
 * @return 					A WorkspaceShell.
 */
[[nodiscard]] std::shared_ptr<WorkspaceSphereShell> paddedSphericalShellAroundLeaves(const AppleTreePlanningScene &scene_info, double padding);

template<typename ShellPoint>
[[nodiscard]]
std::shared_ptr<ArmHorizontalDecorator<ShellPoint>> horizontalAdapter(std::shared_ptr<WorkspaceShell<ShellPoint>> shell) {
	return std::make_shared<ArmHorizontalDecorator<ShellPoint>>(shell);
}

#endif //NEW_PLANNERS_SPHERESHELL_H
