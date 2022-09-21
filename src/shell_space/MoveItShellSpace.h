
#ifndef NEW_PLANNERS_MOVEITSHELLSPACE_H
#define NEW_PLANNERS_MOVEITSHELLSPACE_H

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>

#include <utility>
#include "WorkspaceShell.h"
#include "../RobotPath.h"
#include "../procedural_tree_generation.h"

/**
 *
 * Given a surface point p and a unit vector a, return a configuration c such that:
 *
 * - The arm is straightened out (joint angles 0 except for base revolute joint).
 * - The base rotation $z$-axis by angle $\phi$, picking $\phi$ and $\theta_0$ such that a vector between the first joint
 *   of the arm and $e(c)$ is aligned exactly with $\ArmVec(p)$
 * - The translation of the floating base $\vec{t}$ picked (through forward kinematics) such that end-effector is positioned exactly at $e(c) = p$.
 *
 * @param drone				The robot model.
 * @param desired_ee_pos	The desired end-effector position.
 * @param armvec			The desired arm vector. (Expected to be a unit vector.)
 * @return					The robot state.
 */
moveit::core::RobotState robotStateFromPointAndArmvec(const moveit::core::RobotModelConstPtr &drone,
													  const Eigen::Vector3d &desired_ee_pos,
													  const Eigen::Vector3d &armvec);

/**
 * A Shell state space defined based on a WorkspaceShell and a MoveIt robot model.
 *
 * @tparam ShellPoint 		Type of points on the shell.
 */
template<typename ShellPoint>
class MoveItShellSpace {

private:
	/// Reference to a robot model.
	moveit::core::RobotModelConstPtr robot_model;

	/// Reference to a workspace shell.
	std::shared_ptr<const WorkspaceShell<ShellPoint>> shell;

public:

	/**
	 * Construct a MoveItShellSpace.
	 * @param robotModel 		The robot model.
	 * @param shell 			The workspace shell.
	 */
	MoveItShellSpace(moveit::core::RobotModelConstPtr robotModel, const std::shared_ptr<const WorkspaceShell<ShellPoint>> &shell)
			: robot_model(std::move(robotModel)), shell(shell) {
	}

	/**
	 * For a given point on the workspace shell, return a robot state.
	 *
	 * @param point 		The point on the shell.
	 * @return 				The robot state.
	 */
	moveit::core::RobotState stateFromPoint(const ShellPoint &point) const {

		Eigen::Vector3d armvec = shell->arm_vector(point);
		Eigen::Vector3d surface_point = shell->surface_point(point);

		return robotStateFromPointAndArmvec(robot_model, surface_point, armvec);

	}

	/**
	 * Given points on the workspace shell, return a robot path between them through the shell space.
	 *
	 * @param start 		The start point on the shell.
	 * @param end 			The end point on the shell.
	 * @return 				The robot path.
	 */
	RobotPath shellPath(const ShellPoint &start, const ShellPoint &end) const {

		// Get the path from the workspace shell.
		auto path = shell->path_from_to(start, end);

		// Convert the path to a robot path.
		RobotPath robot_path;

		if (auto piecewise = std::dynamic_pointer_cast<const PiecewiseLinearPath<ShellPoint>>(path)) {
			// Convert points one-to-one.
			for (const auto &point : piecewise->points) {
				robot_path.waypoints.push_back(stateFromPoint(point));
			}
			return robot_path;
		} if (auto curve = std::dynamic_pointer_cast<const CurvePath<ShellPoint>>(path)) {
			// Just take 100 samples.
			for (size_t i = 0; i <= 100; i++) {
				double t = (double) i / 100.0;
				ShellPoint point = curve->at(t);
				robot_path.waypoints.push_back(stateFromPoint(point));
			}
		} else {
			throw std::runtime_error("Unknown path type.");
		}

		return robot_path;
	}

	double predict_path_length(const ShellPoint &start, const ShellPoint &end) const {
		return shell->path_from_to(start, end)->length();
	}

	ShellPoint pointNearState(const moveit::core::RobotState &state) const {
		return shell->nearest_point_on_shell(state.getGlobalLinkTransform("end_effector").translation());
	}

	ShellPoint pointNearGoal(const Apple& apple) const {
		return shell->nearest_point_on_shell(apple.center);
	}

};

#endif //NEW_PLANNERS_MOVEITSHELLSPACE_H
