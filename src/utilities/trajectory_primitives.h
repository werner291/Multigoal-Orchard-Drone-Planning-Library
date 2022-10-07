
#ifndef NEW_PLANNERS_TRAJECTORY_PRIMITIVES_H
#define NEW_PLANNERS_TRAJECTORY_PRIMITIVES_H

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>

robot_trajectory::RobotTrajectory turnInPlace(const moveit::core::RobotState &current_state, double angle);

#endif //NEW_PLANNERS_TRAJECTORY_PRIMITIVES_H
