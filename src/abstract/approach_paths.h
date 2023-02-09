/**
 * Copyright (c) 2022 University College Roosevelt
 */

#ifndef NEW_PLANNERS_APPROACH_PATHS_H
#define NEW_PLANNERS_APPROACH_PATHS_H

#include "path_traits.h"
#include "shell_space.h"
#include <optional>
#include <functional>
#include <vector>
#include <type_traits>

/**
 * An "Approach Path" connecting some arbitrary state to a shell state.
 *
 * By convention, the initial state of the approach path is the shell state,
 * whereas the final state is the arbitrary state to be approached.
 *
 * Furthermore, the `shell_point` member of the approach path is the point
 * on the shell that corresponds to this initial state.
 *
 * @tparam Path
 * @tparam State
 * @tparam Goal
 * @tparam ShellSpace
 * @tparam ApproachPathPlanner
 */
template<typename Path, typename ShellSpace>
struct ApproachPath {
	typename path_traits<Path>::state_type path;
	typename shell_state_traits<ShellSpace>::shell_point shell_point;
};

/**
 * @brief Result struct for the `approach_paths` function.
 */
template<typename Path, typename ShellSpace>
struct ApproachPathsResult {

	/// A vector of approach paths; unreachable goals are omitted.
	std::vector<ApproachPath<Path, ShellSpace>> approach_paths;

	/// For every input goal, this vector contains the index of the corresponding approach path;
	/// unreachable goals are represented as `std::nullopt`.
	std::vector<std::optional<size_t>> goal_to_path_map;
};

/**
 * Computes approach paths for a given set of goals, initial state, shell space, and approach path planner.
 *
 * @tparam Path - The type representing a path.
 * @tparam State - The type representing a state.
 * @tparam Goal - The type representing a goal.
 * @tparam ShellSpace - The type representing the shell space.
 * @tparam ApproachPathPlanner - The type representing the approach path planner. Must be invoked as ApproachPathPlanner(initial_state, goal, shell_space).
 *
 * @param initial_state - The initial state.
 * @param goal_set - The set of goals.
 * @param shell_space - The shell space.
 * @param approach_path_planner - The approach path planner.
 *
 * @return A struct containing the computed approach paths and a map from each goal to its corresponding approach path.
 */
template<typename Path,
        typename State,
		typename Goal,
		typename ShellSpace,
		typename ApproachPathPlanner>
ApproachPathsResult<Path, ShellSpace> approachPaths(const State &initial_state,
												   const std::vector<Goal> &goal_set,
												   const ShellSpace &shell_space,
												   ApproachPathPlanner& approach_path_planner) {

	ApproachPathsResult<Path, ShellSpace> result;

	for (size_t goal_i = 0; goal_i < goal_set.size(); goal_i++) {
		auto goal = goal_set[goal_i];
		auto approach_path = approach_path_planner(initial_state, goal, shell_space);

		if (approach_path) {
			result.approach_paths.push_back(*approach_path);
			result.goal_to_path_map.push_back(result.approach_paths.size() - 1);
		} else {
			result.goal_to_path_map.push_back(std::nullopt);
		}

	}

	return result;

}

#endif //NEW_PLANNERS_APPROACH_PATHS_H
