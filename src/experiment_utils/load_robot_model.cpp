// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/1/23.
//

#include "load_robot_model.h"
#include "../planning/moveit_forward_declarations.h"

#include <moveit/robot_model/robot_model.h>

moveit::core::RobotModelPtr mgodpl::experiment_assets::loadRobotModel(double base_joint_weight) {

	// Get the project root directory, and append test_robots/urdf/bot.urdf

	std::string source_path = MYSOURCE_ROOT;
	auto urdf = std::make_shared<urdf::Model>();
	urdf->initFile(source_path + "/test_robots/urdf/bot.urdf");

	auto srdf = std::make_shared<srdf::Model>();
	srdf->initFile(*urdf, source_path + "/test_robots/config/aerial_manipulator_drone.srdf");

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
