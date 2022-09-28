#ifndef NEW_PLANNERS_ARMHORIZONTAL_H
#define NEW_PLANNERS_ARMHORIZONTAL_H

#include "WorkspaceShell.h"

/**
 * A decorator class for a WorkspaceSphereShell that makes the arm vector always horizontal by projecting it onto the plane and normalizing.
 *
 * @tparam ShellPoint	The type of points on the shell.
 */
template<typename ShellPoint>
class ArmHorizontalDecorator : public WorkspaceShell<ShellPoint> {

	/// The underlying shell.
	std::shared_ptr<WorkspaceShell<ShellPoint>> shell;

public:

	/**
	 * Create a decorator for a WorkspaceSphereShell.
	 * @param shell 		The underlying shell (as a shared pointer).
	 */
	explicit ArmHorizontalDecorator(const std::shared_ptr<WorkspaceShell<ShellPoint>> &shell) : shell(shell) {
	}

	Eigen::Vector3d arm_vector(const ShellPoint &p) const override {

		Eigen::Vector3d arm = shell->arm_vector(p);

		return Eigen::Vector3d(arm.x(), arm.y(), 0.0).normalized();

	}

	ShellPoint nearest_point_on_shell(const Eigen::Vector3d &p) const override {
		return shell->nearest_point_on_shell(p);
	}

	Eigen::Vector3d surface_point(const ShellPoint &p) const override {
		return shell->surface_point(p);
	}

	std::shared_ptr<ShellPath<ShellPoint>> path_from_to(const ShellPoint &from, const ShellPoint &to) const override {
		return shell->path_from_to(from, to);
	}

	ShellPoint random_near(const ShellPoint &p, double radius) const override {
		return shell->random_near(p, radius);
	}

	double path_length(const std::shared_ptr<ShellPath<ShellPoint>> &path) const override {
		return shell->path_length(path);
	}

};

#endif //NEW_PLANNERS_ARMHORIZONTAL_H
