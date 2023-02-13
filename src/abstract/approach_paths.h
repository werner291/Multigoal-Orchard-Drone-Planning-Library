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

namespace abstract {

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
	template<typename Path, typename ShellPoint>
	struct ApproachPath {
		Path path;
		ShellPoint shell_point;
	};

	/**
	 * @brief Result struct for the `approach_paths` function.
	 */
	template<typename Path, typename ShellPoint>
	struct ApproachPathsResult {

		/// A vector of approach paths; unreachable goals are omitted.
		std::vector<ApproachPath<Path, ShellPoint>> approach_paths;

		/// For every input goal, this vector contains the index of the corresponding approach path;
		/// unreachable goals are represented as `std::nullopt`.
		std::vector<std::optional<size_t>> goal_to_path_map;
	};

}

#endif //NEW_PLANNERS_APPROACH_PATHS_H
