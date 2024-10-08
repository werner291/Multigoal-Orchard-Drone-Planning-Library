// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#include "../experiment_utils/procedural_robot_models.h"
#include "../experiment_utils/tree_benchmark_data.h"
#include "../planning/RandomNumberGenerator.h"
#include "../planning/RobotModel.h"
#include "../visualization/RunQueue.h"
#include "../visualization/Throttle.h"
#include "../visualization/robot_state.h"
#include "../visualization/visualization_function_macros.h"

#include <functional>
#include <memory>
#include <thread>
#include <vector>
#include <vtkActor.h>

using namespace mgodpl;
using namespace visualization;
using namespace vizualisation;

import sampling;
import collision_detection;
import collision_visualization;

using namespace mgodpl;
using namespace visualization;

REGISTER_VISUALIZATION(rrt_for_approach_planning) {

	auto robot = experiments::createProceduralRobotModel(
			{
					.total_arm_length = 1.0,
					.joint_types = {
							experiments::JointType::HORIZONTAL
					},
					.add_spherical_wrist = false
			});

	auto tree = experiments::loadBenchmarkTreemodelData("appletree");

	Throttle throttle;

	auto base_collision_fn =
			collision_check_fn_in_environment(
					{robot, *tree.tree_collision_object});

	auto run_queue = std::make_shared<RunQueue>();
	auto actors = std::make_shared<std::vector<RobotActors>>();

	auto motion_check_visualization_fn =
			create_motion_check_visualization_fn(
					base_collision_fn,
					actors,
					run_queue,
					throttle,
					robot);

	random_numbers::RandomNumberGenerator rng;

	auto sample = make_uniform_sampler_fn(robot, rng, tree.tree_mesh.leaves_mesh, 2.0);

	viewer.addTree(tree.tree_mesh, true, true);

	// We're going to be running the algorithm in a separate thread to keep the logic as clean as possible:
	std::thread algorithm_thread([&]() {
		while (true) {
			auto state_1 = sample();
			auto state_2 = sample();
			motion_check_visualization_fn(state_1, state_2);
		}
	});

	viewer.addTimerCallback([&]() {
		run_queue->run_all(viewer);
		throttle.allow_advance();
	});

	viewer.start();

}
