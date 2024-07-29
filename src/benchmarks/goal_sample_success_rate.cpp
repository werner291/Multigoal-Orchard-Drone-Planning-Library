// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 7/26/24.
//

#include <iostream>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include <range/v3/view/transform.hpp>
#include <range/v3/to_container.hpp>
#include <range/v3/view/iota.hpp>

#include "benchmark_function_macros.h"
#include "../experiment_utils/LoadedTreeModel.h"
#include "../experiment_utils/procedural_robot_models.h"

#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/tree_models.h"
#include "../planning/collision_detection.h"
#include "../planning/fcl_utils.h"
#include "../planning/goal_sampling.h"

#include "../planning/RobotModel.h"

#include <execution>

/**
 * Pick k indices without replacement from the range 0..n. (Exclusive of n.)
 *
 * @param n		The number of indices to pick from.
 * @param k		The number of indices to pick.
 * @param rng	The random number generator to use.
 * @return		A vector of k indices.
 */
std::vector<size_t> pick_indices_without_replacement(size_t n, size_t k, std::default_random_engine &rng) {
	assert(n >= k);

	std::vector<size_t> indices(n);
	std::iota(indices.begin(), indices.end(), 0);

	std::shuffle(indices.begin(), indices.end(), rng);

	indices.resize(k);

	return indices;
}

REGISTER_BENCHMARK(goal_sample_success_rate) {
	std::cout << "Running goal_sample_success_rate" << std::endl;

	// Grab a list of all tree models:
	const auto tree_model_names = mgodpl::tree_meshes::getTreeModelNames();

	std::cout << "Will evaluate for the following tree models:";
	for (const auto &tree_model: tree_model_names) {
		std::cout << " " << tree_model;
	}
	std::cout << std::endl;
	std::cout << "Total of " << tree_model_names.size() << " tree models." << std::endl;

	// Note the tree names:
	results["tree_models"] = Json::arrayValue;
	for (const auto &tree_model: tree_model_names) {
		results["tree_models"].append(tree_model);
	}

	// Make CollisionObjectd's for each tree:
	std::vector<fcl::CollisionObjectd> tree_collision_objects;

	// Load the tree meshes for each tree:
	std::vector<mgodpl::tree_meshes::TreeMeshes> tree_meshes;
	for (const auto &tree_model_name: tree_model_names) {
		tree_meshes.push_back(mgodpl::tree_meshes::loadTreeMeshes(tree_model_name));
		std::cout << "Creating collision object for tree model " << tree_model_name << std::endl;
		tree_collision_objects.emplace_back(mgodpl::fcl_utils::meshToFclBVH(tree_meshes.back().trunk_mesh));
	}

	const auto robot_params = mgodpl::experiments::generateRobotArmParameters({
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

	std::default_random_engine shuffler(42); // NOLINT(*-msc51-cpp)

	std::vector<Problem> problems;
	for (size_t tree_model_index = 0; tree_model_index < tree_meshes.size(); ++tree_model_index) {
		if (tree_meshes[tree_model_index].fruit_meshes.size() <= MAX_GOALS_PER_TREE) {
			for (size_t goal_index = 0; goal_index < tree_meshes[tree_model_index].fruit_meshes.size(); ++goal_index) {
				for (size_t robot_model_index = 0; robot_model_index < robot_models.size(); ++robot_model_index) {
					problems.push_back(Problem{
						.tree_model_index = tree_model_index,
						.goal_index = goal_index,
						.robot_model_index = robot_model_index
					});
				}
			}
		} else {
			// Grab the first MAX_GOALS_PER_TREE indices:
			for (size_t goal_index: pick_indices_without_replacement(tree_meshes[tree_model_index].fruit_meshes.size(),
			                                                         MAX_GOALS_PER_TREE,
			                                                         shuffler)) {
				for (size_t robot_model_index = 0; robot_model_index < robot_models.size(); ++robot_model_index) {
					problems.push_back(Problem{
						.tree_model_index = tree_model_index,
						.goal_index = goal_index,
						.robot_model_index = robot_model_index
					});
				}
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
		const auto &tree_mesh = tree_meshes[problem.tree_model_index];

		// Get the robot model:
		const auto &robot_model = robot_models[problem.robot_model_index];

		const auto base_link = robot_model.findLinkByName("flying_base");
		const auto end_effector_link = robot_model.findLinkByName("end_effector");

		// Get the goal:
		const auto &goal = tree_mesh.fruit_meshes[problem.goal_index];

		// AABB center:
		const mgodpl::math::Vec3d goal_center = mgodpl::mesh_aabb(goal).center();

		const auto &tree_collision = tree_collision_objects[problem.tree_model_index];

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
			if (check_robot_collision(robot_model, tree_collision, sample)) {
				collisions += 1;
			}
		}

		// Record end time:
		auto end_time = std::chrono::high_resolution_clock::now();

		Json::Value result;
		result["tree_model"] = tree_model_names[problem.tree_model_index];
		result["goal_index"] = static_cast<int>(problem.goal_index);
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
