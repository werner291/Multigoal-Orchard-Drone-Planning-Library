// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#include "../experiment_utils/procedural_robot_models.h"
#include "../experiment_utils/tree_benchmark_data.h"
#include "../planning/RandomNumberGenerator.h"
#include "../planning/RobotModel.h"
#include "../planning/RobotPath.h"
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
import goal_sampling;
import collision_detection;
import collision_visualization;
import shell_state_projection;
import approach_makeshift_prm;
import rrt;

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
	auto actors = std::make_shared<std::vector<RobotActors> >();

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

	std::function sample_goal_t = goal_region_sampler(robot, rng);


	std::function check_goal_state = visualize_and_cleanup_state(base_collision_fn, run_queue, throttle, robot);

	auto accept_at = accept_outside_tree(*tree.tree_convex_hull);

	// We're going to be running the algorithm in a separate thread to keep the logic as clean as possible:
	std::thread algorithm_thread([&]() {
		for (const auto &target: tree.target_points) {
			std::function sample_goal = [&]() {
				return sample_goal_t(target);
			};

			// Try the RRT operation at valid goal samples.
			auto path = try_at_valid_goal_samples<RobotPath>(
				check_goal_state,
				sample_goal,
				1000,
				[&](const RobotState &goal) -> std::optional<RobotPath> {
					auto shell_state = project_to_shell_state(goal, *tree.tree_convex_hull, robot);

					auto biased_sampler = make_biased_sample_fn(goal,
					                                            shell_state,
					                                            rng,
					                                            1.0);

					// Run the RRT algorithm and try to find a path.
					return rrt_path_to_acceptable(
						goal,
						biased_sampler,
						base_collision_fn,
						motion_check_visualization_fn,
						equal_weights_distance,
						100,
						accept_at
					);
				});
		}
	});

	viewer.addTimerCallback([&]() {
		run_queue->run_all(viewer);
		throttle.allow_advance();
	});

	viewer.start();
}
