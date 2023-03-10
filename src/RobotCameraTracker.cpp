// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10-3-23.
//

#include "RobotCameraTracker.h"

RobotCameraTracker::RobotCameraTracker(vtkCamera *camera, const moveit::core::RobotState &state) : camera(camera) {
	// Initialize robot position
	robot_position = state.getGlobalLinkTransform("base_link").translation();
}

void RobotCameraTracker::update(const moveit::core::RobotState &state) {

	robot_position = robot_position * 0.2 + 0.8 * state.getGlobalLinkTransform("base_link").translation();

	Eigen::Vector3d tree_center(0.0, 0.0, 2.0);

	Eigen::Vector3d relative = robot_position - tree_center;

	relative = Eigen::AngleAxisd(M_PI / 6.0, Eigen::Vector3d::UnitZ()) * relative;

	Eigen::Vector3d cam_pos = tree_center + relative.normalized() * 8.0;
	cam_pos.z() = 4.0;

	camera->SetViewUp(0, 0, 1);
	camera->SetFocalPoint(robot_position.x(), robot_position.y(), robot_position.z());
	camera->SetPosition(cam_pos.x(), cam_pos.y(), cam_pos.z());

}
