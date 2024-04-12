// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 4/12/24.
//

#include "../visualization/visualization_function_macros.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../planning/RandomNumberGenerator.h"
#include "../planning/RobotState.h"
#include "../visualization/robot_state.h"

using namespace mgodpl;

REGISTER_VISUALIZATION(reacharound_motions) {

	// Create a random number generator.
	random_numbers::RandomNumberGenerator rng;

	robot_model::RobotModel robot = experiments::createProceduralRobotModel({
		.total_arm_length = 1.0,
		.joint_types = {experiments::JointType::HORIZONTAL, experiments::JointType::VERTICAL},
	});

	RobotState state {
			.base_tf = math::Transformd::fromTranslation({-1.0, 0.0, 0.0}),
			.joint_values = std::vector<double>(robot.count_joint_variables(), 1.0)
	};

	viewer.addSphere(0.2, {0.0, 0.0, 0.0}, {0.5, 0.5, 0.5});

	auto robot_viz = vizualisation::vizualize_robot_state(viewer, robot, forwardKinematics(robot, state.joint_values, 0, state.base_tf));

	viewer.setCameraTransform({0.0, 8.0, 4.0}, {0.0, 0.0, 0.0});

	viewer.start();

}
