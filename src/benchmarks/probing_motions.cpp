// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-8-24.
//

// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 7/26/24.
//

#include <iostream>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>

#include "benchmark_function_macros.h"
#include "../experiment_utils/LoadedTreeModel.h"
#include "../experiment_utils/procedural_robot_models.h"

#include "../experiment_utils/tree_models.h"
#include "../planning/collision_detection.h"
#include "../planning/goal_sampling.h"

#include "../planning/cgal_chull_shortest_paths.h"
#include "../experiment_utils/tree_benchmark_data.h"
#include "../planning/probing_motions.h"
#include "../planning/approach_path_planning.h"

#include <execution>
#include <CGAL/Side_of_triangle_mesh.h>

using namespace mgodpl;
using namespace experiments;

/**
 * The purpose of this experiment is to measure the success rate
 * of probing motions.
 *
 * A.K.A. the procedure where we first find a goal state,
 * and then pull the robot out of the tree along the axis
 * of the arm, thereby minimizing the chance of collision.
 */
REGISTER_BENCHMARK(probing_motions) {
	std::cout << "Running goal_sample_success_rate" << std::endl;

	// Grab a list of all tree models:
	auto tree_models = mgodpl::experiments::loadAllTreeBenchmarkData(results);

	robot_model::RobotModel robot_model = mgodpl::experiments::createProceduralRobotModel(
			{
					.total_arm_length = 1.0,
					.joint_types = {JointType::HORIZONTAL},
					.add_spherical_wrist = false
			});

	const auto base_link = robot_model.findLinkByName("flying_base");
	const auto end_effector_link = robot_model.findLinkByName("end_effector");

	// We're going to run experiments in parallel, so we need a mutex to protect the results:
	std::mutex results_mutex;

	// We want to now take a carthesian product of all tree models, the goals within them, and the robot models:
	struct Problem {
		size_t tree_model_index;
		size_t goal_index;
	};

	// We get some truly excessive numbers of problems here, so we limit the number of goals per tree:
	const size_t MAX_GOALS_PER_TREE = 100;

	random_numbers::RandomNumberGenerator shuffler(42);

	std::vector<Problem> problems;
	for (size_t tree_model_index = 0; tree_model_index < tree_models.size(); ++tree_model_index) {
		// Grab the first MAX_GOALS_PER_TREE indices:
		for (size_t goal_index: shuffler.pick_indices_without_replacement(tree_models[tree_model_index].tree_mesh.fruit_meshes.size(),
																		  MAX_GOALS_PER_TREE)) {
			problems.push_back(Problem{.tree_model_index = tree_model_index, .goal_index = goal_index,});
		}

	}

	const size_t MAX_SAMPLES = 1000;

	results["max_samples"] = (int) MAX_SAMPLES;

	// Shuffle in a deterministic way:
	std::shuffle(problems.begin(), problems.end(), std::default_random_engine(42)); // NOLINT(*-msc51-cpp)

	// Keep a counter of threads in-flight:
	std::atomic<int> threads_in_flight = 0;

	std::function fn = [&](const Problem &problem) {

		Json::Value result;
		result["tree_model"] = tree_models[problem.tree_model_index].tree_model_name;
		result["goal_index"] = static_cast<int>(problem.goal_index);

		// Increment the number of threads in flight:
		threads_in_flight += 1;

		// Get the tree model:
		const auto &tree_mesh = tree_models[problem.tree_model_index].tree_mesh;

		// Get the goal:
		const auto &goal = tree_mesh.fruit_meshes[problem.goal_index];

		// AABB center:
		const mgodpl::math::Vec3d goal_center = mgodpl::mesh_aabb(goal).center();

		// Look up the distance of the goal to the tree canopy hull:
		const auto &tree_convex_hull = tree_models[problem.tree_model_index].tree_convex_hull;
		CGAL::Side_of_triangle_mesh<mgodpl::cgal::Surface_mesh, mgodpl::cgal::K> inside(tree_convex_hull->convex_hull);
		const auto distance = sqrt(tree_convex_hull->tree.squared_distance(mgodpl::cgal::to_cgal_point(goal_center)));

		// Check whether the base center is inside the convex hull
		bool inside_tree = inside(mgodpl::cgal::to_cgal_point(goal_center)) == CGAL::ON_BOUNDED_SIDE;

		if (!inside_tree) {
			std::cout << "Goal is outside the tree; skipping since there is no point in trying to move out."
					  << std::endl;
			std::lock_guard lock(results_mutex);
			threads_in_flight -= 1;
			return;
		}

		result["signed_goal_depth"] = inside_tree ? distance : -distance;

		const auto &tree_collision = tree_models[problem.tree_model_index].tree_collision_object;

		// RNG:
		random_numbers::RandomNumberGenerator rng(42);

		int collisions = 0;
		int successful_pullouts = 0;

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
				continue;
			} else {
				const auto &path = straightout(
						robot_model,
						sample,
						tree_convex_hull->tree,
						tree_convex_hull->mesh_path
				);

				// Check collisions:
				successful_pullouts += check_path_collides(robot_model, *tree_collision, path.path);
			}

		}

		result["collisions"] = collisions;
		result["successful_pullouts"] = successful_pullouts;

		{
			// Let's do a straight-in motion as well:
			const auto straight_in = approach_planning::straight_in_motion(
					robot_model,
					*tree_convex_hull,
					goal_center
			);

			result["straight_in_collides"] = check_path_collides(robot_model, *tree_collision, straight_in.path);
		}

		// Record end time:
		auto end_time = std::chrono::high_resolution_clock::now();

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

	std::for_each(std::execution::par,
				  problems.begin(),
				  problems.end(),
				  fn);
}
