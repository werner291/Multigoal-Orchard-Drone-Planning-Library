
#ifndef NEW_PLANNERS_STATIC_SHELL_APPROACH_H
#define NEW_PLANNERS_STATIC_SHELL_APPROACH_H

#include <type_traits>
#include <vector>
#include <functional>
#include <optional>
#include <range/v3/all.hpp>

/**
 * @brief Plan a path for a set of goals with a static goal set.
 *
 * @tparam State 				The type representing the state of the system.
 * @tparam Goal 				The type representing the goal of the system.
 * @tparam Path 				The type representing the planned path, default is `std::vector<State>`.
 * @tparam ShellPoint 			The type representing the points on the shell space.
 * @tparam GoalApproachPathPlanner  The type of the function used to plan the approach path to a goal, default is `std::function<std::optional<Path>(const Goal&, const ShellPoint&)>`.
 * @tparam StateApproachPathPlanner The type of the function used to plan the approach path to a state, default is `std::function<std::optional<Path>(const State&, const ShellPoint&)>`.
 * @tparam OnShellPathPlanner	The type of the function used to plan the path between two points on the shell, default is `std::function<Path(const ShellPoint& a, const ShellPoint& b)>`.
 * @tparam TSPPlanner 			The type of the function used to plan a TSP route, default is `std::function<std::vector<int>(const ShellPoint&, const std::vector<ShellPoint&>&)>`.
 * @tparam PathOptimizer 		The type of the function used to optimize the planned path, default is `std::function<Path(const Path&)>`.
 *
 * @param initial_state 		The initial state of the system.
 * @param goal_set 				The set of goals to be achieved.
 * @param plan_approach_path_to_goal The function to plan the approach path to a goal.
 * @param plan_approach_path_to_state The function to plan the approach path to a state.
 * @param plan_on_shell_path	The function to plan the path between two points on the shell.
 * @param tsp_planner 			The TSP planner.
 * @param path_optimizer 		The path optimizer.
 *
 * @return The planned path that takes into account the set of goals and the initial state of the system.
 */
template<typename State, typename Goal, typename Path, typename ShellPoint, typename GoalApproachPathPlanner = std::function<std::optional<Path>(
		const Goal &)>, typename StateApproachPathPlanner = std::function<std::optional<Path>(const State &)>, typename OnShellPathPlanner = std::function<Path(
		const ShellPoint &a,
		const ShellPoint &b)>, typename TSPPlanner = std::function<std::vector<int>(const ShellPoint &,
																					const std::vector<ShellPoint &> &)>,
																							typename PathOptimizer = std::function<Path(
		const Path &)> >
Path plan_path_static_goalset(const State &initial_state,
							  const std::vector<Goal> &goal_set,
							  GoalApproachPathPlanner plan_approach_path_to_goal,
							  StateApproachPathPlanner plan_approach_path_to_state,
							  OnShellPathPlanner plan_on_shell_path,
							  TSPPlanner tsp_planner,
							  PathOptimizer path_optimizer) {

	// Plan an approach path for every goal.
	auto approach_paths = goal_set | ranges::views::transform(plan_approach_path_to_goal) |
						  ranges::views::filter([](const auto &path) { return path.has_value(); }) |
						  ranges::views::transform([](const auto &path) { return path.value(); }) | ranges::to_vector;

	// Plan an approach path from the initial state to the shell.
	auto initial_path = plan_approach_path_to_state(initial_state);

	// Determine the shell points for the goals.
	auto visit_order = tsp_planner(initial_path->shell_point, approach_paths | ranges::views::transform([](const auto &path) {
		return path->shell_point;
	}) | ranges::to_vector);

	// Plan the goal-to-goal paths.
	std::vector<Path> goal_to_goal;
	for (size_t i = 0; i < visit_order.size() - 1; i++) {
		Path gtg = assemble_goal_to_goal_path(approach_paths[visit_order[i]], approach_paths[visit_order[i + 1]]);
		goal_to_goal.push_back(path_optimizer(gtg));
	}

	// Concatenate all the paths into one big one.
	Path planned_path = initial_path.value();
	for (const auto &path: goal_to_goal) {
		planned_path = path_traits<Path>::concatenate(std::move(planned_path), path);
	}

	return planned_path;

}


#endif //NEW_PLANNERS_STATIC_SHELL_APPROACH_H
