// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "JointSpacePath.h"

#include <moveit/robot_model/robot_model.h>

//
// Created by werner on 7-11-23.
//
bool mgodpl::moveit_facade::stateIsOnMotion(const moveit::core::RobotModel &robot,
											const std::array<JointSpacePoint, 2> &motion,
											const JointSpacePoint &point,
											const double threshold) {

	double dist_ab = robot.distance(motion[0].joint_values.data(), motion[1].joint_values.data());
	double dist_ap = robot.distance(motion[0].joint_values.data(), point.joint_values.data());
	double dist_pb = robot.distance(point.joint_values.data(), motion[1].joint_values.data());

	return std::abs(dist_ab - (dist_ap + dist_pb)) < threshold;
}

std::optional <size_t> mgodpl::moveit_facade::findMotionOnPath(const moveit::core::RobotModel &robot,
															   const JointSpacePath &path,
															   const JointSpacePoint &point) {

	for (size_t i = 0; i < path.path.size() - 1; ++i) {
		if (stateIsOnMotion(robot, {path.path[i], path.path[i + 1]}, point)) {
			return i;
		}
	}

	return std::nullopt;
}
