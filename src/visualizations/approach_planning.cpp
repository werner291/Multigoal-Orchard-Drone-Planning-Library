// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 9/12/24.
//

#include <optional>
#include <vector>
#include "../visualization/visualization_function_macros.h"
#include "../visualization/robot_state.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../planning/RandomNumberGenerator.h"
#include "../planning/RobotState.h"

using namespace mgodpl;

import approach_makeshift_prm;

REGISTER_VISUALIZATION(nearby_sampling) {

    // In this visualization, we will show the process of sampling nearby states and states along a motion.
    const auto &robot = experiments::createProceduralRobotModel({
		.total_arm_length = 1.0,
		.joint_types = {experiments::JointType::HORIZONTAL},
	});

	RobotState start_state {
		.base_tf = math::Transformd::identity(),
		.joint_values = {0.0}
	};

	auto initial_fk = robot_model::forwardKinematics(robot, start_state);

	mgodpl::vizualisation::vizualize_robot_state(viewer, robot, initial_fk, {1, 0, 1});

    random_numbers::RandomNumberGenerator rng(42);

	const double FACTOR = 0.5;

	int samples = 0;

	std::optional<vizualisation::RobotActors> last_actors;

	viewer.addTimerCallback([&](){

			RobotState nearby_state = sample_nearby_state(start_state, rng, FACTOR);

			auto fk = robot_model::forwardKinematics(robot, nearby_state);


			if (last_actors) {
				for (auto &actor : last_actors->actors) {
					viewer.removeActor(actor);
				}
			}

			// Draw the nearby state:
			last_actors = mgodpl::vizualisation::vizualize_robot_state(viewer, robot, fk, {0, 1, 0});

	});

	viewer.lockCameraUp();

	viewer.setCameraTransform({5,5,1},{0,0,0});

    viewer.start();
}


REGISTER_VISUALIZATION(makeshift_exponential_sampling) {

	// In this visualization, we will show the process of sampling nearby states and states along a motion.
	const auto &robot = experiments::createProceduralRobotModel({
																		.total_arm_length = 1.0,
																		.joint_types = {experiments::JointType::HORIZONTAL},
																});

	RobotState start_state {
			.base_tf = math::Transformd::identity(),
			.joint_values = {0.0}
	};

	RobotState end_state {
			.base_tf = {
					.translation = {1, 1, 1},
					.orientation = math::Quaterniond::fromAxisAngle(math::Vec3d::UnitZ(), 1.0)
					},
			.joint_values = {M_PI_2}
	};

	auto initial_fk = robot_model::forwardKinematics(robot, start_state);

	for (int i = 0; i <= 10; ++i) {
		double t = i / 10.0;
		// Interpolate between the start and end state:
		RobotState interpolated_state = interpolate(start_state, end_state, t);
		auto fk = robot_model::forwardKinematics(robot, interpolated_state);
		mgodpl::vizualisation::vizualize_robot_state(viewer, robot, fk, {1, 0, 1});
	}

	random_numbers::RandomNumberGenerator rng(42);

	const double FACTOR = 2.0;

	int samples = 0;

	std::vector<vizualisation::RobotActors> last_actors;
	const size_t KEEP = 10;

	viewer.addTimerCallback([&](){

		RobotState nearby_state = makeshift_exponential_sample_along_motion(start_state, end_state, rng, FACTOR);

		auto fk = robot_model::forwardKinematics(robot, nearby_state);

		while (last_actors.size() >= KEEP) {
			for (auto &actor : last_actors.front().actors) {
				viewer.removeActor(actor);
			}
			last_actors.erase(last_actors.begin());
		}

		// Draw the nearby state:
		last_actors.push_back(mgodpl::vizualisation::vizualize_robot_state(viewer, robot, fk, {0, 1, 0}));

	});

	viewer.lockCameraUp();

	viewer.setCameraTransform({5,5,1},{0,0,0});

	viewer.start();
}