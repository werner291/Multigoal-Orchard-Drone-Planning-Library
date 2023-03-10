// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10-3-23.
//

#ifndef NEW_PLANNERS_TRAJECTORYTIMING_H
#define NEW_PLANNERS_TRAJECTORYTIMING_H

#include <moveit/robot_trajectory/robot_trajectory.h>

/**
 * Calculates the maximum distance between corresponding active joints in the current and previous RobotState.
 *
 * Whether this function is appropriate or the RobotState::distance() function is appropriate depends on the use case;
 * considering that joints can move simultaneously, this function works best for measuring time taken to move from one
 * state to the other, while RobotState::distance() works best as a measure of the total amount of change between two
 * states, perhaps as a measure of the amount of effort/energy required to move from one state to the other.
 *
 * @param a The current RobotState.
 * @param b The previous RobotState.
 *
 * @return The maximum joint distance between corresponding active joints in the current and previous RobotState.
 */
void maxjointDistance(const moveit::core::RobotState &a, const moveit::core::RobotState &b);

void adjustTiming(robot_trajectory::RobotTrajectory &trajectory);

#endif //NEW_PLANNERS_TRAJECTORYTIMING_H
