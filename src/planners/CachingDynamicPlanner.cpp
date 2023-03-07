// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include "CachingDynamicPlanner.h"
#include "../probe_retreat_move.h"
#include "../utilities/vector_utils.h"
#include "../utilities/ompl_tools.h"

template<typename ShellPoint>
std::optional<DynamicMultiGoalPlanner::PathSegment>
CachingDynamicPlanner<ShellPoint>::replan_after_removal(const ompl::base::SpaceInformationPtr &si,
														const ompl::base::State *current_state,
														const ompl::base::GoalPtr &removed_goal,
														const PathInterrupt &interrupt,
														const AppleTreePlanningScene &planning_scene) {

	size_t removed_index;
	for (removed_index = 0; removed_index < ordering.size(); removed_index++) {
		if (ordering[removed_index].goal == removed_goal) {
			break;
		}
	}

	assert(removed_index < ordering.size());

	auto new_ordering = tsp_method->update_ordering_with_removal(ordering.size(),
																 removed_index,
																 [&](const size_t &a, const size_t &b) -> double {
																	 return shell_space->predict_path_length(ordering[a]
																													 .approach
																													 .shell_point,
																											 ordering[b]
																													 .approach
																													 .shell_point);
																 },
																 [&](const size_t &a) -> double {
																	 return shell_space->predict_path_length(ordering[a]
																													 .approach
																													 .shell_point,
																											 to_shell_cache
																													 ->shell_point);
																 });

	ordering = reorder_by_index(ordering, new_ordering);

	return continueFromInterrupt(si, current_state, interrupt);
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

	// If we successfully found a path to the new goal, we can add it to the ordering.
	if (approach) {

		auto new_ordering = determine_new_ordering_with_insertion(*approach);

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

	return continueFromInterrupt(si, current_state, interrupt);

}

template<typename ShellPoint>
std::optional<DynamicMultiGoalPlanner::PathSegment>
CachingDynamicPlanner<ShellPoint>::continueFromInterrupt(const ompl::base::SpaceInformationPtr &si,
														 const ompl::base::State *current_state,
														 const PathInterrupt &interrupt) {

	if (this->last_emitted_path->goal == this->ordering.front().goal) {

		utilities::truncatePathToInterrupt(this->last_emitted_path->path, interrupt);

		return this->last_emitted_path->path;

	} else {

		// Find a path to the shell. We probably need to recompute the to_shell_cache path since we got interrupted.
		this->to_shell_cache = this->find_path_to_shell(si, current_state);

		// If we couldn't find a path to the shell, then the robot must be stuck somewhere. Halt.
		if (!this->to_shell_cache) {

			// TODO: we could probably do something better here, such as re-using the last approach path?
			// Or, maybe try harder/over?
			// Might not work since we're following an optimized path.
			std::cout << "Could not find a way back to the shell." << std::endl;

			return std::nullopt;

		} else {
			std::cout << "Did find a way back to the shell." << std::endl;
		}

		// Most likely the next path segment will be requested after visiting the first goal
		// in the ordering, so that's the approach path to which we'll cache.
		// TODO	to_shell_cache = ordering[0].approach;

		auto path = this->optimizedPointToPoint(si, *this->to_shell_cache, this->ordering.front().approach);

		this->last_emitted_path = {{path, this->ordering.front().goal}};

		return path;
	}
}

template<typename ShellPoint>
std::vector<IncrementalTSPMethods::NewOrderingEntry>
CachingDynamicPlanner<ShellPoint>::determine_new_ordering_with_insertion(const OmplApproachPath<ShellPoint> &approach) const {

	auto lookup_index = [&](const IncrementalTSPMethods::NewOrderingEntry &a) {
		if (std::holds_alternative<IncrementalTSPMethods::NewGoal>(a)) {
			return approach.shell_point;
		} else {
			return ordering[std::get<IncrementalTSPMethods::FromOriginal>(a).index].approach.shell_point;
		}
	};

	auto distance_1 = [&](const IncrementalTSPMethods::NewOrderingEntry &a) {
		return shell_space->predict_path_length(to_shell_cache->shell_point, lookup_index(a));
	};

	auto distance_2 = [&](const IncrementalTSPMethods::NewOrderingEntry &a,
						  const IncrementalTSPMethods::NewOrderingEntry &b) {
		return shell_space->predict_path_length(lookup_index(a), lookup_index(b));
	};

	// Determine the new ordering indices using the TSP method.
	return tsp_method->update_ordering_with_insertion(ordering.size(), distance_2, distance_1);
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
std::optional<DynamicMultiGoalPlanner::PathSegment>
CachingDynamicPlanner<ShellPoint>::replan_after_path_end(const ompl::base::SpaceInformationPtr &si,
														 const ompl::base::State *current_state,
														 const AppleTreePlanningScene &planning_scene) {

	assert(!ordering.empty());

	// Find a path to the shell. We didn't get interrupted, so the cached path should be good.
	auto to_shell = find_path_to_shell(si, current_state);

	if (!to_shell) {
		// TODO: I'm curious if this branch is ever taken, since it's weird to be at the end of a path
		// and not have the cache be valid at this point in time.
		std::cout << "Could not find a way back to the shell." << std::endl;
		return std::nullopt;
	}

	// In case of the CachedDynamicPlanner, all emitted paths end in a goal state.
	// Thus, if we are at the end of a path, we can safely remove the first goal from the ordering
	// since it must just have been visited.
	ordering.erase(ordering.begin());

	// If the ordering is empty, there is no need to move further. Return nullopt to signal that the planner is done.
	if (ordering.empty()) {

		// Clear the last emitted path since it's no longer valid.
		last_emitted_path.reset();

		// Return nullopt since we're halding.
		return std::nullopt;

	} else {

		// The currently-returned path ends in the goal first in the ordering. So, the next path segment
		// will likely start there, and we can cache the approach path to that goal as a starting point
		// for that next path segment.
		to_shell_cache = ordering[0].approach;

		// Plan a path to the shell (using our retreat path) and then to the next goal.
		auto path = optimizedPointToPoint(si, *to_shell, ordering[0].approach);

		// Store the path we're about to emit as the last emitted path.
		last_emitted_path = {{path, ordering.front().goal}};

		// Return the path.
		return path;
	}
}

template<typename ShellPoint>
std::optional<DynamicMultiGoalPlanner::PathSegment>
CachingDynamicPlanner<ShellPoint>::plan_initial(const ompl::base::SpaceInformationPtr &si,
												const ompl::base::State *start,
												const std::vector<ompl::base::GoalPtr> &initial_goals,
												const AppleTreePlanningScene &planning_scene,
												double padding) {

	// Initialize our shell space now that we have the scene.
	shell_space = shellBuilder(planning_scene, si);

	// Compute a path from wherever the robot is to the shell.
	auto to_shell = find_path_to_shell(si, start);

	if (!to_shell) {
		// If this returns nullopt, the robot might be stuck. Still, it'd be weird, since this should be open space.
		std::cout << "Could not find a way from the initial state to the shell." << std::endl;
		return std::nullopt;
	}

	// Compute an approach path for each goal.
	for (const auto &goal: initial_goals) {
		// Compute the path.
		auto approach = approach_planner->approach_path(goal, *shell_space);

		if (approach) {
			// Successfully computed an approach path. Add it to the ordering.
			// We just insert them at the back, we'll sort them later.
			ordering.push_back({goal, *approach});
		}
		// Else, the goal is considered to be unreachable. By the nature of sampling-based planners,
		// this is not guaranteed to be true, but we must draw the line somewhere.
	}

	// If the ordering is empty, there is no need to move further. Return nullopt to signal that the planner is done.
	if (ordering.empty()) {
		return std::nullopt;
	}

	// Find the indices by which to sort the ordering.
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

	// Most likely the next path segment will be requested after visiting the first goal
	// in the ordering, so that's the approach path to which we'll cache.
	to_shell_cache = ordering[0].approach;

	// Plan a path to the shell from the start state to the first goal via the shell, and optimize it.
	auto ptp = optimizedPointToPoint(si, *to_shell, ordering.front().approach);

	// Make sure the path starts at the start state.
	assert(si->distance(ptp.getState(0), start) < 1e-6);

	// Store the path we're about to emit as the last emitted path.
	last_emitted_path = {{ptp, ordering.front().goal}};

	// Return the path.
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