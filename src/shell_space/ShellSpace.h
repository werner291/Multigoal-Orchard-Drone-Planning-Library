
#ifndef NEW_PLANNERS_SHELLSPACE_H
#define NEW_PLANNERS_SHELLSPACE_H

#include <moveit/robot_state/robot_state.h>

/**
 * Describes, in MoveIt terms, a "shell" shape that fits around the obstacles in the scene,
 * and can be used to quickly plan collision-free paths around then.
 */
template<typename ShellPoint>
class ShellSpace {

public:
	/**
	 * Construct a RobotState located on the collision-free shell,
	 * in a point corresponding to the given shell point.
	 *
	 * @param drone 	A robot model.
	 * @param a 		The shell point (assumed to be on the shell)
	 * @return 			A RobotState located on the collision-free shell.
	 */
	[[nodiscard]] virtual moveit::core::RobotState state_on_shell(
			const moveit::core::RobotModelConstPtr &drone,
			const ShellPoint &a) const = 0;

	/**
	 * Construct a path from a to b, on the collision-free shell.
	 *
	 * @param drone 	A robot model (assumed to be on the shell)
	 * @param a 		The start point (assumed to be on the shell).
	 * @param b 		The end point (assumed to be on the shell).
	 * @return 			The path.
	 */
	[[nodiscard]] virtual std::vector<moveit::core::RobotState> path_on_shell(
			const moveit::core::RobotModelConstPtr &drone,
			const ShellPoint &a,
			const ShellPoint &b) const = 0;

	/**
	 * Sample a point within a (roughly) Gaussian distribution around the given point on the sphere.
	 *
	 * @param near 	The point to sample near.
	 * @return 		The sampled point.
	 */
	[[nodiscard]] virtual ShellPoint gaussian_sample_near_point(const ShellPoint& near) const = 0;

	/**
	 * Predict the length of a path from a to b, on the collision-free shell, without constructing the path.
	 *
	 * This prediction may be an approximation.
	 *
	 * @param a 		The start point (assumed to be on the shell).
	 * @param b 		The end point (assumed to be on the shell).
	 * @return 			The predicted length.
	 */
	[[nodiscard]] virtual double predict_path_length(
			const ShellPoint &a,
			const ShellPoint &b) const = 0;

	/**
	 * Project a RobotState onto the collision-free shell.
	 *
	 * @param st 		The RobotState to project.
	 * @return 			The projected point.
	 */
	[[nodiscard]] virtual ShellPoint project(const moveit::core::RobotState& st) const = 0;

	/**
	 * Project an Apple/target onto the collision-free shell.
	 * @param st 		The Apple to project.
	 * @return 			The projected point.
	 */
	[[nodiscard]] virtual ShellPoint project(const Apple& st) const = 0;

};

#endif //NEW_PLANNERS_SHELLSPACE_H
