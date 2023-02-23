// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

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

	auto approach = approach_planner->approach_path(new_goal, *shell_space);

	// Must recompute the to_shell path since we got interrupted.
	to_shell = find_path_to_shell(si, current_state);

	if (!to_shell) {

		// TODO: we could probably do something better here, such as re-using the last approach path?
		// Or, maybe try harder/over?
		// Might not work since we're following an optimized path.
		std::cout << "Could not find a way back to the shell." << std::endl;

		return std::nullopt;

	} else {
		std::cout << "Did find a way back to the shell." << std::endl;
	}

	if (approach) {

		// Find the least costly insertion point, which is either:
		// 1. At the start of the order
		// 2. Between two existing goals
		// 3. At the end of the order

		// For now, assume it's at the start of the order
		size_t insert_before = -1;
		double best_insertion_cost = std::numeric_limits<double>::infinity();

		for (size_t i = 0; i < ordering.size(); i++) {

			ShellPoint approach_before = i == 0 ? to_shell->shell_point : ordering[i - 1].approach.shell_point;
			std::optional<ShellPoint> approach_after =
					i == ordering.size() - 1 ? std::nullopt : std::optional(ordering[i + 1].approach.shell_point);

			double insertion_cost = shell_space->predict_path_length(approach_before, approach->shell_point) +
									(approach_after ? shell_space->predict_path_length(approach->shell_point,
																					   *approach_after) : 0.0) -
									(approach_after ? shell_space->predict_path_length(approach_before, *approach_after)
													: 0.0);

			if (insertion_cost < best_insertion_cost) {
				best_insertion_cost = insertion_cost;
				insert_before = i;
			}

		}

		ordering.insert(ordering.begin() + insert_before, {new_goal, *approach});
	}

	return PathSegment{SIZE_MAX, optimizedPointToPoint(si, *to_shell, ordering.front().approach)};

}

template<typename ShellPoint>
std::optional<DynamicMultiGoalPlanner::PathSegment> CachingDynamicPlanner<ShellPoint>::replan_after_successful_visit(
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
		return PathSegment{SIZE_MAX, optimizedPointToPoint(si, *to_shell, ordering[0].approach)};
	}
}

template<typename ShellPoint>
std::optional<DynamicMultiGoalPlanner::PathSegment>
CachingDynamicPlanner<ShellPoint>::plan(const ompl::base::SpaceInformationPtr &si,
										const ompl::base::State *start,
										const std::vector<ompl::base::GoalPtr> &initial_goals,
										const AppleTreePlanningScene &planning_scene) {
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

	batch_reorder(*to_shell);

	auto ptc = ompl::base::plannerNonTerminatingCondition();

	return PathSegment{SIZE_MAX, // TODO Get the IDs right.
					   optimizedPointToPoint(si, *to_shell, ordering.front().approach)};
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
void CachingDynamicPlanner<ShellPoint>::batch_reorder(const OmplApproachPath<ShellPoint> &initial_approach) {

	auto indices = tsp_method->initial_ordering(ordering.size(), [&](size_t i, size_t j) {
		// Use the shell space to predict the path length between each pair of approach paths
		return shell_space->predict_path_length(ordering[i].approach.shell_point, ordering[j].approach.shell_point);
	}, [&](size_t i) {
		// Use the shell space to predict the path length between the initial approach path
		// and each approach path in the list
		return shell_space->predict_path_length(initial_approach.shell_point, ordering[i].approach.shell_point);
	});

	// Use the TSP method to reorder the approach paths
	ordering = reorder_by_index(std::move(ordering), indices);
}


template<typename ShellPoint>
CachingDynamicPlanner<ShellPoint>::CachingDynamicPlanner(const std::shared_ptr<ApproachPlanningMethods<ShellPoint>> &approachPlanner,
														 const std::shared_ptr<IncrementalTSPMethods> &tspMethod,
														 MkOmplShellFn<ShellPoint> shellBuilder) :
		approach_planner(approachPlanner), tsp_method(tspMethod), shellBuilder(shellBuilder) {

}

template
class CachingDynamicPlanner<Eigen::Vector3d>;