
#ifndef NEW_PLANNERS_MOVEIT_H
#define NEW_PLANNERS_MOVEIT_H

#include <moveit/robot_state/robot_state.h>
#include <Eigen/Core>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/collision_detection_fcl/collision_env_fcl.h>

/**
 * Assuming the robot has a floating base as the first 7 joint variables, this function sets the translation of the base.
 *
 * @param st 			The robot state to set the base translation for.
 * @param offset 		The translation to set the base to.
 */
void setBaseTranslation(moveit::core::RobotState &st, const Eigen::Vector3d &offset);

Eigen::Vector3d getBaseTranslation(const moveit::core::RobotState &robotState);

/**
 * Assuming the robot has a floating base as the first 7 joint variables, this function sets the orientation of the base.
 *
 * @param robotState 	The robot state to set the base orientation for.
 * @param q 			The orientation to set the base to.
 */
void setBaseOrientation(moveit::core::RobotState &robotState, const Eigen::Quaterniond &q);

Eigen::Quaterniond getBaseOrientation(const moveit::core::RobotState &robotState);

/**
 * Set the given robot state to a point at time T in the given trajectory.
 *
 * @param state 				The robot state to set.
 * @param t 					The time T to set the robot state to.
 * @param currentTrajectory 	The trajectory to get the robot state from.
 */
void setStateToTrajectoryPoint(moveit::core::RobotState &state,
							   double t,
							   const robot_trajectory::RobotTrajectory &currentTrajectory);

/**
 * A very simple function that checks whether the given robot state is in collision, and nothing else.
 *
 * @param current_state 		The robot state to check.
 * @param collision_env 		The collision environment to check against.
 * @return 						True if the robot state is in collision, false otherwise.
 */
[[nodiscard]] bool checkCollision(const moveit::core::RobotState &current_state,
					const collision_detection::CollisionEnvFCL &collision_env);

/**
 * Returns a changed robot state where the end-effector is moved to the given position.
 *
 * @param state 				The robot state to change.
 * @param position 				The position to move the end-effector to.
 * @param endEffectorName 		The name of the end-effector link
 * @return 						A new robot state where the end-effector is moved to the given position.
 */
[[nodiscard]] moveit::core::RobotState setEndEffectorToPosition(moveit::core::RobotState state,
																const Eigen::Vector3d &position,
																const std::string &endEffectorName = "end_effector");

moveit::core::RobotState sampleStateNearByUpright(const moveit::core::RobotState& st, double distance);

/**
 * A small class that can be fed subsequent samples of robot states over time,
 * and can be used to determine useful statistics
 */
class RobotPastTrace {
	/// A number of last-known positions of the robot, from oldest to newest, assumed to be about spaced evenly in time.
	std::vector<moveit::core::RobotState> last_robot_states;

	size_t keep_count;

public:

	/**
	 * Constructor.
	 *
	 * @param keep_count 		The number of last-known positions to keep.
	 * @param initial_state 		The initial state of the robot.
	 */
	RobotPastTrace(size_t keep_count, const moveit::core::RobotState &initial_state);

	/**
	 * Add a new robot state to the trace. Will discard the oldest state if the trace is full in a ring-buffer-like fashion.
	 *
	 * @param state 		The new robot state to add.
	 */
	void addRobotState(const moveit::core::RobotState &robot_state);

	/**
	 * @return 	The last-added robot state.
	 */
	[[nodiscard]] const moveit::core::RobotState &lastRobotState() const;

	/**
	 * @brief Return the i-th most recently-added element.
	 */
	[[nodiscard]] const moveit::core::RobotState &fromBack(size_t i) const;

	/**
	 * @brief Return the distance between the last two states, or 0 if there are less than two states.
	 */
	[[nodiscard]] double lastStepSize() const;
};


#endif //NEW_PLANNERS_MOVEIT_H
