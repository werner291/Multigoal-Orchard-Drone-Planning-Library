// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include "CachingDynamicPlanner.h"
#include "../probe_retreat_move.h"
#include "../utilities/vector_utils.h"

template<typename ShellPoint>
std::optional<DynamicMultiGoalPlanner::PathSegment>
CachingDynamicPlanner<ShellPoint>::replan_after_removal(const ompl::base::SpaceInformationPtr &si,
														const ompl::base::State *current_state,
														const ompl::base::GoalPtr &removed_goal,
														const PathInterrupt &interrupt,
														const AppleTreePlanningScene &planning_scene) {

	throw std::runtime_error("Not implemented");

	return {};
}

template<typename ShellPoint>
std::optional<DynamicMultiGoalPlanner::PathSegment>
CachingDynamicPlanner<ShellPoint>::replan_after_discovery(const ompl::base::SpaceInformationPtr &si,
														  const ompl::base::State *current_state,
														  const ompl::base::GoalPtr &new_goal,
														  const PathInterrupt &interrupt,
														  const AppleTreePlanningScene &planning_scene) {

	// Plan a path to the new goal.
	auto approach = approach_planner->approach_path(new_goal, *shell_space);

	// Find a path to the shell. We probably need to recompute the to_shell_cache path since we got interrupted.
	to_shell_cache = find_path_to_shell(si, current_state);

	// If we couldn't find a path to the shell, then the robot must be stuck somewhere. Halt.
	if (!to_shell_cache) {

		// TODO: we could probably do something better here, such as re-using the last approach path?
		// Or, maybe try harder/over?
		// Might not work since we're following an optimized path.
		std::cout << "Could not find a way back to the shell." << std::endl;

		return std::nullopt;

	} else {
		std::cout << "Did find a way back to the shell." << std::endl;
	}

	// If we successfully found a path to the new goal, we can add it to the ordering.
	if (approach) {

		// Determine the new ordering indices using the TSP method.
		auto new_ordering = tsp_method->update_ordering(ordering.size(),
														[&](const IncrementalTSPMethods::NewOrderingEntry &a,
															const IncrementalTSPMethods::NewOrderingEntry &b) {

															// Look up the ShellPoint of either the existing goal in the ordering, or the new goal.
															ShellPoint a_sp = std::holds_alternative<IncrementalTSPMethods::NewGoal>(
																	a) ? approach->shell_point
																	   : ordering[std::get<IncrementalTSPMethods::FromOriginal>(
																			a).index].approach.shell_point;

															// Same for b.
															ShellPoint b_sp = std::holds_alternative<IncrementalTSPMethods::NewGoal>(
																	b) ? approach->shell_point
																	   : ordering[std::get<IncrementalTSPMethods::FromOriginal>(
																			b).index].approach.shell_point;

															return shell_space->predict_path_length(a_sp, b_sp);

														},
														[&](const IncrementalTSPMethods::NewOrderingEntry &a) {

															// Same again, but only for one of the goals; the other is simply the ShellPoint closest to the robot.
															ShellPoint a_sp = std::holds_alternative<IncrementalTSPMethods::NewGoal>(
																	a) ? approach->shell_point
																	   : ordering[std::get<IncrementalTSPMethods::FromOriginal>(
																			a).index].approach.shell_point;

															return shell_space->predict_path_length(to_shell_cache->shell_point,
																									a_sp);

														});

		{// Validate the new ordering: must only have indices to the existing ordering and the new goal.
			assert(std::all_of(new_ordering.begin(),
							   new_ordering.end(),
							   [&](const IncrementalTSPMethods::NewOrderingEntry &e) {
								   return std::holds_alternative<IncrementalTSPMethods::NewGoal>(e) ||
										  std::get<IncrementalTSPMethods::FromOriginal>(e).index < ordering.size();
							   }));

			// NewGoal may appear exactly once
			assert(std::count_if(new_ordering.begin(),
								 new_ordering.end(),
								 [](const IncrementalTSPMethods::NewOrderingEntry &e) {
									 return std::holds_alternative<IncrementalTSPMethods::NewGoal>(e);
								 }) == 1);

			// All indices must be unique
			std::vector<bool> indices(ordering.size(), false);
			assert(std::all_of(new_ordering.begin(),
							   new_ordering.end(),
							   [&](const IncrementalTSPMethods::NewOrderingEntry &e) {
								   if (std::holds_alternative<IncrementalTSPMethods::NewGoal>(e)) {
									   return true;
								   } else {
									   auto index = std::get<IncrementalTSPMethods::FromOriginal>(e).index;
									   if (indices[index]) {
										   return false;
									   } else {
										   indices[index] = true;
										   return true;
									   }
								   }
							   }));
		}

		// Build a new ordering by moving out of the old ordering, according to the given order.
		ordering = new_ordering |
				   ranges::views::transform([&](const IncrementalTSPMethods::NewOrderingEntry &e) -> ApproachToGoal {

					   if (std::holds_alternative<IncrementalTSPMethods::NewGoal>(e)) {
						   // If the entry is the new goal, then we'll use the approach path we just computed.
						   return ApproachToGoal{new_goal, *approach // TODO consider using std::move
						   };
					   } else {
						   // Otherwise, we'll just move the existing entry.
						   return ordering[std::get<IncrementalTSPMethods::FromOriginal>(e).index]; // TODO consider using std::move
					   }

				   }) | ranges::to_vector;
	}

	// Most likely the next path segment will be requested after visiting the first goal
	// in the ordering, so that's the approach path to which we'll cache.
	// TODO	to_shell_cache = ordering[0].approach;

	return optimizedPointToPoint(si, *to_shell_cache, ordering.front().approach);

}

template<typename ShellPoint>
std::optional<OmplApproachPath<ShellPoint>>
CachingDynamicPlanner<ShellPoint>::find_path_to_shell(const ompl::base::SpaceInformationPtr &si,
													  const ompl::base::State *start) {

	if (to_shell_cache.has_value() && si->distance(start, to_shell_cache->robot_path.getStates().back()) < 1.0e-6) {
		std::cout << "Using cached approach path." << std::endl;
		return to_shell_cache;
	} else {
		to_shell_cache = approach_planner->approach_path(start, *shell_space);
		return to_shell_cache;
	}

}

template<typename ShellPoint>
std::optional<DynamicMultiGoalPlanner::PathSegment> CachingDynamicPlanner<ShellPoint>::replan_after_path_end(
		const ompl::base::SpaceInformationPtr &si,
		const ompl::base::State *current_state,
		const AppleTreePlanningScene &planning_scene) {

	if (ordering.empty()) {
		return std::nullopt;
	}

	auto to_shell = find_path_to_shell(si, current_state);

	if (!to_shell) {
		// TODO: We have an approach path here to use.

		std::cout << "Could not find a way back to the shell." << std::endl;
		return std::nullopt;
	}

	ordering.erase(ordering.begin());

	if (ordering.empty()) {
		return std::nullopt;
	} else {

		// Most likely the next path segment will be requested after visiting the first goal
		// in the ordering, so that's the approach path to which we'll cache.
		to_shell_cache = ordering[0].approach;

		return optimizedPointToPoint(si, *to_shell, ordering[0].approach);
	}
}

template<typename ShellPoint>
std::optional<DynamicMultiGoalPlanner::PathSegment>
CachingDynamicPlanner<ShellPoint>::plan_initial(const ompl::base::SpaceInformationPtr &si,
												const ompl::base::State *start,
												const std::vector<ompl::base::GoalPtr> &initial_goals,
												const AppleTreePlanningScene &planning_scene,
												double padding) {
	shell_space = shellBuilder(planning_scene, si);

	auto to_shell = find_path_to_shell(si, start);

	if (!to_shell) {
		std::cout << "Could not find a way from the initial state to the shell." << std::endl;
		return std::nullopt;
	}

	for (const auto &goal: initial_goals) {
		auto approach = approach_planner->approach_path(goal, *shell_space);

		if (approach) {
			ordering.push_back({goal, *approach});
		}
	}

	if (ordering.empty()) {
		return std::nullopt;
	}

	auto indices = tsp_method->initial_ordering(ordering.size(), [&](size_t i, size_t j) {
		// Use the shell space to predict the path length between each pair of approach paths
		return shell_space->predict_path_length(ordering[i].approach.shell_point, ordering[j].approach.shell_point);
	}, [&](size_t i) {
		// Use the shell space to predict the path length between the initial approach path
		// and each approach path in the list
		return shell_space->predict_path_length((*to_shell).shell_point, ordering[i].approach.shell_point);
	});

	// Use the TSP method to reorder the approach paths
	ordering = reorder_by_index(std::move(ordering), indices);

	auto ptc = ompl::base::plannerNonTerminatingCondition();

	// Most likely the next path segment will be requested after visiting the first goal
	// in the ordering, so that's the approach path to which we'll cache.
	to_shell_cache = ordering[0].approach;

	auto ptp = optimizedPointToPoint(si, *to_shell, ordering.front().approach);

	assert(si->distance(ptp.getState(0), start) < 1e-6);

	return ptp;
}

template<typename ShellPoint>
ompl::geometric::PathGeometric
CachingDynamicPlanner<ShellPoint>::optimizedPointToPoint(const ompl::base::SpaceInformationPtr &si,
														 const OmplApproachPath<ShellPoint> &retreat_path,
														 const OmplApproachPath<ShellPoint> &approach_path) const {

	// Create a non-terminating condition for the planner
	auto ptc = ompl::base::plannerNonTerminatingCondition();

	// Compute the shell path
	auto shell_path = this->shell_space->shellPath(retreat_path.shell_point, approach_path.shell_point);

	// Concatenate the retreat, shell, and approach paths to form a complete point-to-point path
	ompl::geometric::PathGeometric path(si);
	path.append(retreat_path.robot_path);
	path.reverse(); // Reverse the retreat path so that it goes from the goal to the start
	path.append(shell_path);
	path.append(approach_path.robot_path);

	// Optimize the path and return the result
	return optimize(path, {nullptr}, si);
}

template<typename ShellPoint>
CachingDynamicPlanner<ShellPoint>::CachingDynamicPlanner(const std::shared_ptr<ApproachPlanningMethods<ShellPoint>> &approachPlanner,
														 const std::shared_ptr<IncrementalTSPMethods> &tspMethod,
														 MkOmplShellFn<ShellPoint> shellBuilder) :
		approach_planner(approachPlanner), tsp_method(tspMethod), shellBuilder(shellBuilder) {

}

template
class CachingDynamicPlanner<Eigen::Vector3d>;