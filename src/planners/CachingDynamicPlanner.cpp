// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "CachingDynamicPlanner.h"
#include "../probe_retreat_move.h"

/**
 * Reorders a vector by the given indexes.
 *
 * Note: Elements are moved from the original vector to the new vector,
 * so duplicate indexes are not allowed and may cause undefined behavior.
 *
 * @tparam T 				The type of the vector elements
 * @param vec 				The vector to reorder (pass by move)
 * @param ordering_indexes 	The indexes to reorder by (must be unique)
 * @return 					The reordered vector
 */
template<typename T>
std::vector<T> reorder_by_index(std::vector<T> vec, const std::vector<size_t> &ordering_indexes) {
	std::vector<T> result;

	for (auto i: ordering_indexes) {
		result.push_back(std::move(vec[i]));
	}

	return result;
}


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

	auto approach = approach_planner->plan(new_goal, *shell_space);

	if (approach) {

		// Find the least costly insertion point, which is either:
		// 1. At the start of the order
		// 2. Between two existing goals
		// 3. At the end of the order

		// For now, assume it's at the start of the order
		size_t insert_before = -1;
		double best_insertion_cost = std::numeric_limits<double>::infinity();

		for (size_t i = 0; i < ordering.size(); i++) {

			ShellPoint approach_before = i == 0 ? to_shell.shell_point : ordering[i - 1].approach.shell_point;
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

	return optimizedPointToPoint(si, to_shell, ordering.front());

}

template<typename ShellPoint>
std::optional<DynamicMultiGoalPlanner::PathSegment> CachingDynamicPlanner<ShellPoint>::replan_after_successful_visit(
		const ompl::base::SpaceInformationPtr &si,
		const ompl::base::State *current_state,
		const ompl::base::GoalPtr &visited_goal,
		const AppleTreePlanningScene &planning_scene) {

	auto ptc = ompl::base::plannerNonTerminatingCondition();

	to_shell = std::move(ordering[0]);

	ordering.erase(ordering.begin());

	if (ordering.empty()) {
		return std::nullopt;
	} else {
		return optimizedPointToPoint(si, ptc, to_shell, ordering[0]);
	}
}

template<typename ShellPoint>
std::optional<DynamicMultiGoalPlanner::PathSegment>
CachingDynamicPlanner<ShellPoint>::plan(const ompl::base::SpaceInformationPtr &si,
										const ompl::base::State *start,
										const std::vector<ompl::base::GoalPtr> &initial_goals,
										const AppleTreePlanningScene &planning_scene) {
	shell_space = shellBuilder(planning_scene, si);

	// This is essentially a re-implementation of ShellPathPlanner::plan(), but caching all the approach paths.

	to_shell = approach_planner->approach_path(start, *shell_space);

	assert(to_shell.has_value());

	for (const auto &goal: initial_goals) {
		auto approach = approach_planner->plan(goal, *shell_space);

		if (approach) {
			ordering.push_back({goal, *approach});
		}
	}

	if (ordering.empty()) {
		return std::nullopt;
	}

	ordering = batch_reorder(to_shell);

	auto ptc = ompl::base::plannerNonTerminatingCondition();

	return optimizedPointToPoint(si, ptc, to_shell, ordering.front());
}

template<typename ShellPoint>
ompl::geometric::PathGeometric
CachingDynamicPlanner<ShellPoint>::optimizedPointToPoint(const ompl::base::SpaceInformationPtr &si,
														 const OmplApproachPath<ShellPoint> &retreat_path,
														 const OmplApproachPath<ShellPoint> &approach_path) const {

	auto ptc = ompl::base::plannerNonTerminatingCondition();

	auto shell_path = this->shell_space->plan(retreat_path, approach_path, ptc);

	ompl::geometric::PathGeometric path(si);

	path.append(retreat_path);
	path.reverse();

	path.append(shell_path);
	path.append(approach_path);

	return optimize(path, {nullptr}, si);

}

template<typename ShellPoint>
void CachingDynamicPlanner<ShellPoint>::batch_reorder(const OmplApproachPath<ShellPoint> &initial_approach) {
	ordering = reorder_by_index(std::move(ordering),
								tsp_method->initial_ordering(ordering.size(), [&](size_t i, size_t j) {
									return shell_space->predict_path_length(ordering[i].approach.shell_point,
																			ordering[j].approach.shell_point);
								}, [&](size_t i) {
									return shell_space->predict_path_length(initial_approach.shell_point,
																			ordering[i].approach.shell_point);
								}));
}

template<typename ShellPoint>
CachingDynamicPlanner<ShellPoint>::CachingDynamicPlanner(const std::shared_ptr<ApproachPlanningMethods<ShellPoint>> &approachPlanner,
														 const std::shared_ptr<IncrementalTSPMethods> &tspMethod,
														 MkOmplShellFn<ShellPoint> shellBuilder) : approach_planner(
		approachPlanner), tsp_method(tspMethod), shellBuilder(shellBuilder) {

}

template<>
class CachingDynamicPlanner<Eigen::Vector3d>;