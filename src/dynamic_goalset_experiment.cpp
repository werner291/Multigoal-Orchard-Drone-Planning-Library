// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <range/v3/algorithm/count_if.hpp>
#include <range/v3/numeric/accumulate.hpp>
#include "dynamic_goalset_experiment.h"
#include "utilities/experiment_utils.h"
#include "planners/DynamicMultiGoalPlannerOmplToMoveitAdapter.h"
#include "DynamicGoalVisitationEvaluation.h"


Json::Value runDynamicPlannerExperiment(const moveit::core::RobotModelPtr &robot, const Experiment &experiment) {

	// *Somewhere* in the state space is something that isn't thread-safe despite const-ness.
	// So, we just re-create the state space every time just to be safe.
	auto ss = omplStateSpaceForDrone(robot);

	// Collision-space is "thread-safe" by using locking. So, if we want to get any speedup at all,
	// we'll need to copy this for every thread
	auto si = loadSpaceInformation(ss, experiment.problem->second.scene);

	// Allocate the planner.
	auto ompl_planner = experiment.planner->second(si);

	// Wrap it into the adapter that lets us use it with MoveIt types.
	auto adapter = std::make_shared<DynamicMultiGoalPlannerOmplToMoveitAdapter>(ompl_planner, si, ss);

	// Create the evaluation object.
	DynamicGoalVisitationEvaluation eval(adapter,
										 experiment.problem->second.start_state,
										 experiment.problem->second.scene,
										 experiment.problem->second.apple_discoverability,
										 *experiment.problem->second.can_see_apple);

	// Record the start time.
	auto start_time = std::chrono::high_resolution_clock::now();

	auto stats_at_start = getDiscoveryStatusStats(eval.getDiscoveryStatus());

	// Run the planner for the initial set of goals.
	eval.runTillCompletion();

	// Record the end time.
	auto end_time = std::chrono::high_resolution_clock::now();

	int n_visited = (int) ranges::count_if(eval.getDiscoveryStatus(),
										   [](const auto &status) { return status == utilities::VISITED; });

	double total_path_length = ranges::accumulate(
			eval.getSolutionPathSegments() | ranges::views::transform([](const auto &segment) {
				return segment.path.length();
			}), 0.0);

	std::chrono::nanoseconds total_time = ranges::accumulate(
			eval.getSolutionPathSegments() | ranges::views::transform([](const auto &segment) {
				return segment.time;
			}), std::chrono::nanoseconds(0));

	Json::Value result;
	result["initial_knowledge"] = toJSON(stats_at_start);
	result["n_visited"] = n_visited;
	result["time"] = std::chrono::duration_cast<std::chrono::milliseconds>(total_time).count();
	result["total_path_length"] = total_path_length;
	result["solution_segments"] = Json::Value(Json::arrayValue);

	const auto solution = eval.getSolutionPathSegments();

	if (!solution.empty()) {

		for (const auto &segment: solution) {

			Json::Value segment_json;
			segment_json["time"] = std::chrono::duration_cast<std::chrono::milliseconds>(segment.time).count();
			segment_json["path_length"] = segment.path.length();

			segment_json["end_event"] = toJSON(segment.goal_event);

			result["solution_segments"].append(segment_json);

		}

	}

	return result;
}

Json::Value toJSON(const Experiment &experiment) {
	Json::Value result;
	result["planner"] = experiment.planner->first;
	result["problem"] = experiment.problem->first;
	return result;
}
