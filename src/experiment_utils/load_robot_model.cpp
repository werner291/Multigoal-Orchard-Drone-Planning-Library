// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/1/23.
//

#include "load_robot_model.h"

#include <moveit_core/moveit/robot_model/robot_model.h>

moveit::core::RobotModelPtr mgodpl::experiment_assets::loadRobotModel(double base_joint_weight) {

	auto urdf = std::make_shared<urdf::Model>();
	urdf->initFile("test_robots/urdf/bot.urdf");

	if (urdf->getName() == "") {
		std::cerr << "Failed to load the robot model. Typically, this is because the assets are in the wrong place. Ensure that the working directory is set to the root of the project containing the 'test_robots' folder." << std::endl;
		throw std::runtime_error("Failed to load robot model; is the path correct?");
	}

	auto srdf = std::make_shared<srdf::Model>();
	srdf->initFile(*urdf, "test_robots/config/aerial_manipulator_drone.srdf");

	auto robot = std::make_shared<moveit::core::RobotModel>(urdf, srdf);

	// By default, the joint distance factor gets set to the dimension count of the joint, see:
	//     https://github.com/ros-planning/moveit/blob/fd36674cc327962aaf27925ddf1ba9c6a8667d35/moveit_core/robot_model/src/robot_model.cpp#L969
	// I have no idea why this is, but I don't like it.
	for (auto &item: robot->getActiveJointModels())
		item->setDistanceFactor(1.0);

	// If it crashes here, it's because the file isn't found.
	robot->getJointModel("world_joint")->setDistanceFactor(base_joint_weight);

	return robot;
}
