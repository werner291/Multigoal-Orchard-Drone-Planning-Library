//
// Created by werner on 9-2-23.
//

#ifndef NEW_PLANNERS_DYNAMIC_SHELL_APPROACH_H
#define NEW_PLANNERS_DYNAMIC_SHELL_APPROACH_H

#include <functional>
#include <optional>
#include "path_traits.h"

#include "goal_to_goal_via_shell.h"

namespace passive_update_planner {
	template<typename State, typename Goal, typename ShellSpace, typename Path, typename ApproachPathPlanner = std::function<std::optional<Path>(
			const State &,
			const Goal &,
			const ShellSpace &)>, typename TSPPlanner = std::function<std::vector<int>(const std::vector<Path> &,
																					   const std::vector<bool> &)>, typename PathOptimizer = std::function<Path(
			const Path &)> >
	struct DynamicGoalsetUpdatablePath {

	};

	/**
	 * @brief Plan a path for a set of goals with a static goal set.
	 *
	 * @tparam State 				The type of the state.
	 * @tparam Goal 				The type of the goal.
	 * @tparam ShellSpace 			The type of the shell space.
	 * @tparam Path 				The type of the path, default is std::vector<State>.
	 * @tparam ApproachPathPlanner  The type of the approach path planner, default is std::function<Path(const State&, const Goal&, const ShellSpace&)>.
	 * @tparam TSPPlanner 			The type of the TSP planner, default is std::function<std::vector<int>(const std::vector<Path>&, const std::vector<bool>&)>.
	 * @tparam PathOptimizer 		The type of the path optimizer, default is std::function<Path(const Path&)>.
	 *
	 * @param initial_state 		The initial state.
	 * @param goal_set 				The set of goals.
	 * @param shell_space 			The shell space.
	 * @param approach_path_planner The approach path planner.
	 * @param tsp_planner 			The TSP planner.
	 * @param path_optimizer 		The path optimizer.
	 *
	 * @return The planned path.
	 */
	template<typename State, typename Goal, typename ShellSpace, typename Path, typename ApproachPathPlanner = std::function<std::optional<Path>(
			const State &,
			const Goal &,
			const ShellSpace &)>, typename TSPPlanner = std::function<std::vector<int>(const std::vector<Path> &,
																					   const std::vector<bool> &)>, typename PathOptimizer = std::function<Path(
			const Path &)> >
	Path plan_path_dynamic_goalset_initial(const State &initial_state,
										   const std::vector<Goal> &goal_set,
										   const ShellSpace &shell_space,
										   ApproachPathPlanner approach_path_planner,
										   TSPPlanner tsp_planner,
										   PathOptimizer path_optimizer) {

		auto approach_paths = approachPaths<Path>(initial_state, goal_set, shell_space, approach_path_planner);
		auto initial_path = approach_path_planner(initial_state, shell_space, shell_space);
		auto visit_order = tsp_planner(approach_paths, initial_path);

		std::vector<Path> optimized_paths;
		for (size_t i = 0; i < visit_order.size() - 1; i++) {

			Path gtg = assemble_goal_to_goal_path(approach_paths[visit_order[i]],
												  shell_space,
												  approach_paths[visit_order[i + 1]]);

			optimized_paths.push_back(path_optimizer(gtg));
		}

		// Step 7: Then finally, concatenate all paths together into a path Pi0: Pi=LOpt(I∗B0∗A'0)∗Pi,i+1 for i=0 to i=|A'|−2.
		Path planned_path = initial_path.value();
		for (const auto &path: optimized_paths) {
			path_traits<Path>::concatenate(planned_path, path);
		}

		return planned_path;

	}
}

#endif //NEW_PLANNERS_DYNAMIC_SHELL_APPROACH_H
