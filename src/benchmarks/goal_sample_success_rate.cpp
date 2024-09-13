// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 7/26/24.
//

#include <iostream>


#include "benchmark_function_macros.h"
#include "../experiment_utils/LoadedTreeModel.h"
#include "../experiment_utils/procedural_robot_models.h"

#include "../experiment_utils/tree_models.h"
#include "../planning/collision_detection.h"
#include "../planning/fcl_utils.h"
#include "../planning/goal_sampling.h"

#include "../planning/RobotModel.h"
#include "../planning/cgal_chull_shortest_paths.h"
#include "../experiment_utils/tree_benchmark_data.h"

#include <execution>

REGISTER_BENCHMARK(goal_sample_success_rate) {
	std::cout << "Running goal_sample_success_rate" << std::endl;

	// Grab a list of all tree models and load them:
	auto tree_models = mgodpl::experiments::loadAllTreeBenchmarkData(results);

	const auto robot_params = mgodpl::experiments::generateRobotArmParameters(
			{
					.arm_lengths = {0.25, 0.5, 0.75, 1.0},
					.max_links = 3,
					.include_all_horizontal = true,
					.include_all_vertical = true,
					.include_alternating_horizontal_vertical = true
			});

	std::cout << "Will use robots: ";
	for (const auto &robot_param: robot_params) {
		std::cout << robot_param.short_designator() << " ";
		results["robot_models"].append(robot_param.short_designator());
	}
	std::cout << "total: " << robot_params.size() << " robots." << std::endl;

	// Generate a list of robot models:
	std::vector<mgodpl::robot_model::RobotModel> robot_models;
	for (const auto &params: robot_params) {
		robot_models.emplace_back(mgodpl::experiments::createProceduralRobotModel(params));
	}
	std::cout << "Created " << robot_models.size() << " robot models." << std::endl;

	// We're going to run experiments in parallel, so we need a mutex to protect the results:
	std::mutex results_mutex;

	// We want to now take a carthesian product of all tree models, the goals within them, and the robot models:
	struct Problem {
		size_t tree_model_index;
		size_t goal_index;
		size_t robot_model_index;
	};

	// We get some truly excessive numbers of problems here, so we limit the number of goals per tree:
	const size_t MAX_GOALS_PER_TREE = 100;

	random_numbers::RandomNumberGenerator shuffler(42);

	std::vector<Problem> problems;
	for (size_t tree_model_index = 0; tree_model_index < tree_models.size(); ++tree_model_index) {
		for (size_t goal_index: shuffler.pick_indices_without_replacement(tree_models[tree_model_index].tree_mesh.fruit_meshes.size(),
																		  MAX_GOALS_PER_TREE)) {
			for (size_t robot_model_index = 0; robot_model_index < robot_models.size(); ++robot_model_index) {
				problems.push_back(Problem{
						.tree_model_index = tree_model_index,
						.goal_index = goal_index,
						.robot_model_index = robot_model_index
				});
			}
		}
	}

	const size_t MAX_SAMPLES = 1000;

	results["max_samples"] = MAX_SAMPLES;

	// Shuffle in a deterministic way:
	std::shuffle(problems.begin(), problems.end(), std::default_random_engine(42)); // NOLINT(*-msc51-cpp)

	// Keep a counter of threads in-flight:
	std::atomic<int> threads_in_flight = 0;

	std::function fn = [&](const Problem &problem) {
		// Increment the number of threads in flight:
		threads_in_flight += 1;

		// Get the tree model:
		const auto &tree_mesh = tree_models[problem.tree_model_index].tree_mesh;

		// Get the robot model:
		const auto &robot_model = robot_models[problem.robot_model_index];

		const auto base_link = robot_model.findLinkByName("flying_base");
		const auto end_effector_link = robot_model.findLinkByName("end_effector");

		// Get the goal:
		const auto &goal = tree_mesh.fruit_meshes[problem.goal_index];

		// AABB center:
		const mgodpl::math::Vec3d goal_center = mgodpl::mesh_aabb(goal).center();

		// Look up the distance of the goal to the tree canopy hull:
		const auto &tree_convex_hull = tree_models[problem.tree_model_index].tree_convex_hull;
		const auto distance = sqrt(tree_convex_hull->tree.squared_distance(mgodpl::cgal::to_cgal_point(goal_center)));

		const auto &tree_collision = tree_models[problem.tree_model_index].tree_collision_object;

		// RNG:
		random_numbers::RandomNumberGenerator rng(42);

		int collisions = 0;

		// Record start time:
		auto start_time = std::chrono::high_resolution_clock::now();

		// Take 1000 goal samples:
		for (size_t i = 0; i < MAX_SAMPLES; ++i) {
			auto sample = mgodpl::genGoalStateUniform(
					rng,
					goal_center,
					robot_model,
					base_link,
					end_effector_link
			);

			// Check collisions:
			if (check_robot_collision(robot_model, *tree_collision, sample)) {
				collisions += 1;
			}
		}

		// Record end time:
		auto end_time = std::chrono::high_resolution_clock::now();

		Json::Value result;
		result["tree_model"] = tree_models[problem.tree_model_index].tree_model_name;
		result["goal_index"] = static_cast<int>(problem.goal_index);
		result["goal_depth"] = distance;
		result["robot_model"] = robot_params[problem.robot_model_index].short_designator();
		result["collisions"] = collisions;
		result["time_ms"] = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

		// Lock the results mutex:
		std::lock_guard lock(results_mutex);
		results["results"].append(result);

		std::cout << "Finished problem " << results["results"].size() << " of " << problems.size() <<
				  std::endl;
		std::cout << "In flight: " << threads_in_flight << std::endl;

		// Decrement the number of threads in flight:
		threads_in_flight -= 1;
	};

	// We're going to run experiments in parallel, so we need a mutex to protect the results:
	std::mutex problems_mutex;
	std::for_each(std::execution::par,
				  problems.begin(),
				  problems.end(),
				  fn);
}
