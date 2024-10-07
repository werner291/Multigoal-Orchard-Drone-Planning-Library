// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#include <thread>
#include "../visualization/visualization_function_macros.h"
#include "../visualization/Throttle.h"
#include "../visualization/robot_state.h"

#include "../experiment_utils/procedural_robot_models.h"

#include "../planning/RandomNumberGenerator.h"
#include "../experiment_utils/tree_benchmark_data.h"

using namespace mgodpl;
using namespace visualization;
using namespace vizualisation;

import sampling;
import collision_detection;

struct CollisionCheckedState {
	RobotState state;
	bool collision;
};

REGISTER_VISUALIZATION(rrt_for_approach_planning) {

	auto robot = experiments::createProceduralRobotModel({
		.total_arm_length = 1.0,
		.joint_types = {
			experiments::JointType::HORIZONTAL
		},
		.add_spherical_wrist = false
	});

	auto tree = experiments::loadBenchmarkTreemodelData("appletree");

	std::mutex mutex;

	Throttle throttle;

	auto base_collision_fn = collision_check_fn_in_environment({
		robot,
		*tree.tree_collision_object
	});

	std::optional<CollisionCheckedState> state_to_visualize;

	auto motion_check_fn = motion_collision_check_fn_from_state_collision([&](const RobotState &state) {

		bool collides = base_collision_fn(state);

		{
			std::lock_guard lock(mutex);
			state_to_visualize = CollisionCheckedState{state, collides};
		}

		throttle.wait_and_advance();
		return collides;
	});

	random_numbers::RandomNumberGenerator rng;

	auto sample = make_uniform_sampler_fn(robot, rng, tree.tree_mesh.leaves_mesh, 2.0);

	viewer.addTree(tree.tree_mesh, true, true);

	// We're going to be running the algorithm in a separate thread to keep the logic as clean as possible:
	std::thread algorithm_thread([&]() {
		while (true) {
			auto state_1 = sample();
			auto state_2 = sample();
			motion_check_fn(state_1, state_2);
		}
	});

	std::vector<RobotActors> actors;

	viewer.addTimerCallback([&]() {

		std::lock_guard lock(mutex);
		if (state_to_visualize) {
			actors.push_back(vizualize_robot_state(viewer, robot, forwardKinematics(robot, state_to_visualize->state),
								  state_to_visualize->collision ? math::Vec3d(1.0, 0.0, 0.0) : math::Vec3d(0.0, 1.0, 0.0)
								  ));
			// Clear the state so we don't keep visualizing it:
			state_to_visualize.reset();
		}

		w

		throttle.allow_advance();

	});

	viewer.start();

}
