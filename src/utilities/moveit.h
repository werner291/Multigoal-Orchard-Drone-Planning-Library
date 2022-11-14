
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

/**
 * Assuming the robot has a floating base as the first 7 joint variables, this function sets the orientation of the base.
 *
 * @param robotState 	The robot state to set the base orientation for.
 * @param q 			The orientation to set the base to.
 */
void setBaseOrientation(moveit::core::RobotState &robotState, const Eigen::Quaterniond &q);

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
bool checkCollision(const moveit::core::RobotState &current_state,
					const collision_detection::CollisionEnvFCL &collision_env);

#endif //NEW_PLANNERS_MOVEIT_H
