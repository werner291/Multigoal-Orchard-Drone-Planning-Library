// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#include <execution>
#include <random>
#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>

#include "benchmark_function_macros.h"
#include "../experiment_utils/tree_benchmark_data.h"
#include "../planning/RobotPath.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../planning/RandomNumberGenerator.h"

import approach_by_pullout;
#include <CGAL/Side_of_triangle_mesh.h>

using namespace mgodpl;

struct ApproachPlanningProblem {
	const robot_model::RobotModel &robot;
	const experiments::TreeModelBenchmarkData &tree_model;
};

struct ApproachPlanningResults {
	std::vector<std::optional<RobotPath>> paths;
};

using ApproachPlanningMethodFn = std::function<ApproachPlanningResults(const ApproachPlanningProblem &,
																	   random_numbers::RandomNumberGenerator &)>;

ApproachPlanningMethodFn probing_by_pullout() {
	return [](const ApproachPlanningProblem &problem,
			  random_numbers::RandomNumberGenerator &rng) -> ApproachPlanningResults {

		std::vector<std::optional<RobotPath>> paths;

		for (const auto &target: problem.tree_model.target_points) {
			auto ap = approach_planning::plan_approach_by_pullout(
					*problem.tree_model.tree_collision_object,
					*problem.tree_model.tree_convex_hull,
					target,
					0.1,
					problem.robot,
					rng,
					1000
			);

			if (ap) {
				paths.emplace_back(ap->path);
			} else {
				paths.emplace_back(std::nullopt);
			}
		}

		return {
				.paths = paths
		};
	};
}

REGISTER_BENCHMARK(approach_planning_comparison) {

	robot_model::RobotModel robot_model = mgodpl::experiments::createProceduralRobotModel(
			{
					.total_arm_length = 1.0,
					.joint_types = {experiments::JointType::HORIZONTAL},
					.add_spherical_wrist = false
			});

	// Grab a list of all tree models:
	auto tree_models = experiments::loadAllTreeBenchmarkData(results);

	// Drop all targets already outside the tree:
	for (auto &tree_model: tree_models) {
		CGAL::Side_of_triangle_mesh<cgal::Surface_mesh, cgal::K> inside_outside_check(
				tree_model.tree_convex_hull->convex_hull);

		erase_if(tree_model.target_points, [&](const math::Vec3d &target) {
			return inside_outside_check(cgal::to_cgal_point(target)) == CGAL::ON_UNBOUNDED_SIDE;
		});
	}

	const size_t REPETITIONS = 10;

	const std::vector<ApproachPlanningMethodFn> methods{
			probing_by_pullout()
	};

	std::vector<ApproachPlanningProblem> problems;
	problems.reserve(tree_models.size());
	for (const auto &tree_model: tree_models) {
		problems.emplace_back(ApproachPlanningProblem{robot_model, tree_model});
	}

	struct Run {
		size_t method_index;
		size_t problem_index;
		size_t repetition_index;
	};

	std::vector<Run> runs;
	for (size_t method_index = 0; method_index < methods.size(); method_index++) {
		for (size_t problem_index = 0; problem_index < problems.size(); problem_index++) {
			for (size_t repetition_index = 0; repetition_index < REPETITIONS; repetition_index++) {
				runs.push_back({method_index, problem_index, repetition_index});
			}
		}
	}

	std::shuffle(runs.begin(), runs.end(), std::mt19937(42));

	std::mutex results_mutex;
	std::atomic_int in_flight = 0;

	std::for_each(std::execution::par, runs.begin(), runs.end(), [&](const Run &run) {
		in_flight += 1;

		// Grab the RNG, seed it with the repetition index:
		random_numbers::RandomNumberGenerator rng(run.repetition_index);

		std::cout << "Starting run " << run.method_index << " " << run.problem_index << " " << run.repetition_index
				  << std::endl;
		std::cout << "In flight: " << in_flight << std::endl;

		const auto &method = methods[run.method_index];
		const auto &problem = problems[run.problem_index];

		// Record start time:
		auto start_time = std::chrono::high_resolution_clock::now();
		const auto &result = method(problem, rng);
		// Record end time:
		auto end_time = std::chrono::high_resolution_clock::now();

		// Time elapsed in milliseconds:
		auto time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

		Json::Value result_json;
		result_json["method"] = run.method_index;
		result_json["problem"] = run.problem_index;
		result_json["repetition"] = run.repetition_index;
		result_json["time_ms"] = time_ms;

		{

			std::lock_guard lock(results_mutex);
			results["results"].append(result_json);

			std::cout << "Finished run " << run.method_index << " " << run.problem_index << " " << run.repetition_index
					  << ", completed " << results["results"].size() << " of " << runs.size() << std::endl;
		}

		in_flight -= 1;
	});
}