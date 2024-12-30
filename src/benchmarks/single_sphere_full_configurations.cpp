// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#include "../planning/RobotPath.h"

#include "benchmark_function_macros.h"
#include "../visualization/visualization_function_macros.h"
#include "../planning/RobotModel.h"
#include "../planning/RandomNumberGenerator.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../visualization/scannable_points.h"
#include "../visualization/robot_state.h"

using namespace mgodpl;

REGISTER_VISUALIZATION(single_start_straight_arm_single_target) {
	// Create a random number generator.
	random_numbers::RandomNumberGenerator rng;

	robot_model::RobotModel robot = experiments::createProceduralRobotModel({
		.total_arm_length = 1.0,
		.joint_types = {
			experiments::JointType::HORIZONTAL,
		},
		.add_spherical_wrist = false
	});

	RobotState state{
		.base_tf = math::Transformd::fromTranslation({-1.0, 0.0, 0.0}),
		.joint_values = std::vector<double>(robot.count_joint_variables(), 1.0)
	};

	visualization::vizualize_robot_state(viewer, robot, state);

	viewer.start();
}
