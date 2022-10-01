
#ifndef NEW_PLANNERS_MOVEIT_H
#define NEW_PLANNERS_MOVEIT_H

#include <moveit/robot_state/robot_state.h>
#include <Eigen/Core>

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
void setBaseOrientation(moveit::core::RobotState &robotState, Eigen::Quaterniond &q);

#endif //NEW_PLANNERS_MOVEIT_H
