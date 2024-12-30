// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#include "../visualization/visualization_function_macros.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../planning/RandomNumberGenerator.h"
#include "../planning/state_tools.h"
#include "../visualization/robot_state.h"

using namespace mgodpl;

REGISTER_VISUALIZATION(arm_axis_motions) {

	// Load a robot model.
	auto robot = experiments::createProceduralRobotModel({
																.total_arm_length = 1.0,
																.joint_types = {
																		experiments::JointType::HORIZONTAL},
														});

	// Create a random number generator.
	random_numbers::RandomNumberGenerator rng;

	for (int i = 0; i <= 100; ++i) {
		double t = i / 100.0;
		auto state1 = fromEndEffectorAndVector(robot, {1.0, 0, t-0.5}, {1, 0, 0});
		auto fk1 = forwardKinematics(robot, state1.joint_values, 0, state1.base_tf);
		auto robot_viz = visualization::vizualize_robot_state(viewer, robot, fk1);
	}

	for (int i = 0; i <= 100; ++i) {
		double t = i / 100.0;
		auto state1 = fromEndEffectorAndVector(robot, {-1.0 - t, 0.0, 0.0}, {1, 0, 0});
		auto fk1 = forwardKinematics(robot, state1.joint_values, 0, state1.base_tf);
		auto robot_viz = visualization::vizualize_robot_state(viewer, robot, fk1);
	}


	viewer.setCameraTransform({0.0, 8.0, 4.0}, {0.0, 0.0, 0.0});
	viewer.lockCameraUp();

	viewer.start();

}
