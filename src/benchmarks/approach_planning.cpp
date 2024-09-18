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

using namespace mgodpl;

struct ApproachPlanningProblem {
	const experiments::TreeModelBenchmarkData &tree_model;
};

struct ApproachPlanningResults {
	std::vector<std::optional<RobotPath>> paths;
};

struct ApproachPlanningResultStats {

	struct PathStats {
		bool is_collision_free;
		double length;
	};

};

using ApproachPlanningMethodFn = std::function<ApproachPlanningResults(const ApproachPlanningProblem &)>;

REGISTER_BENCHMARK(approach_planning) {

	// Grab a list of all tree models:
	auto tree_models = experiments::loadAllTreeBenchmarkData(results);

	const size_t REPETITIONS = 10;

	const std::vector<ApproachPlanningMethodFn> methods{
			[](const ApproachPlanningProblem &problem) -> ApproachPlanningResults {
				return {
						.paths = {}
				};
			}
	};

	std::vector<ApproachPlanningProblem> problems;
	problems.reserve(tree_models.size());
	for (const auto &tree_model: tree_models) {
		problems.emplace_back(ApproachPlanningProblem{tree_model});
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

		std::cout << "Starting run " << run.method_index << " " << run.problem_index << " " << run.repetition_index
				  << std::endl;
		std::cout << "In flight: " << in_flight << std::endl;

		const auto &method = methods[run.method_index];
		const auto &problem = problems[run.problem_index];
		const auto &result = method(problem);

		Json::Value result_json;
		result_json["method"] = run.method_index;
		result_json["problem"] = run.problem_index;
		result_json["repetition"] = run.repetition_index;

		{

			std::lock_guard lock(results_mutex);
			results["results"].append(result_json);

			std::cout << "Finished run " << run.method_index << " " << run.problem_index << " " << run.repetition_index
					  << ", completed " << results["results"].size() << " of " << runs.size() << std::endl;
		}

		in_flight -= 1;
	});
}