// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10-3-23.
//

#include "trajectoryTiming.h"

double maxJointDistance(const moveit::core::RobotState &current, const moveit::core::RobotState &previous) {
	double max_distance = 0.0;

	// Iterate through each active joint and calculate the distance between the corresponding joints in the current and previous RobotStates.
	for (const moveit::core::JointModel *joint: current.getRobotModel()->getActiveJointModels()) {
		double d = current.distance(previous, joint);
		// If the calculated distance is greater than the current maximum distance, update the maximum distance.
		if (d > max_distance) {
			max_distance = d;
		}
	}

	return max_distance;
}

void adjustTiming(robot_trajectory::RobotTrajectory &trajectory) {

	// Adjust timing on the trajectory.
	for (size_t waypoint_i = 1; waypoint_i < trajectory.getWayPointCount(); waypoint_i++) {

		double max_distance = maxJointDistance(trajectory.getWayPoint(waypoint_i),
											   trajectory.getWayPoint(waypoint_i - 1));

		trajectory.setWayPointDurationFromPrevious(waypoint_i, max_distance);

	}
}
