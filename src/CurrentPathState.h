
#ifndef NEW_PLANNERS_CURRENTPATHSTATE_H
#define NEW_PLANNERS_CURRENTPATHSTATE_H

#include <optional>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

/**
 * A simple class to keep track of the current path (if any) and progress along that path
 */
struct CurrentPathState {

	/// Current progress along the path
	double pathProgressT = 0.0;

	/// The current state of the robot
	std::optional<robot_trajectory::RobotTrajectory> currentTrajectory;

	/// The current state of the robot
	moveit::core::RobotState current_state;

	/**
	 * @brief Return the current state of the robot.
	 */
	[[nodiscard]] const moveit::core::RobotState &getCurrentState() const;

	/**
	 * Create a new CurrentPathState with the given initial state.
	 *
	 * @param initialState 		The initial state of the robot.
	 */
	explicit CurrentPathState(const moveit::core::RobotState &initialState);

	/**
	 * Set the current path to the given trajectory and set the progress to 0.
	 *
	 * @param trajectory 		The new trajectory to follow.
	 */
	void newPath(const robot_trajectory::RobotTrajectory &trajectory);

	/**
	 * Advance the current path by the given time.
	 *
	 * @param dt 				The time to advance the path by.
	 */
	void advance(double dt);

};

#endif //NEW_PLANNERS_CURRENTPATHSTATE_H
