//
// Created by werner on 9-2-23.
//

#ifndef NEW_PLANNERS_TSP_ON_SHELL_H
#define NEW_PLANNERS_TSP_ON_SHELL_H

#include <vector>
#include <cstddef>
#include "path_traits.h"

#include "../utilities/traveling_salesman.h"
#include "shell_space.h"
#include "approach_paths.h"

/**
 * @brief Given a vector of approach paths, and an initial approach path,
 *        return a vector of indices of the approach paths in the order
 *        they should be visited.
 *
 * @tparam Path 		The type of the path.
 * @tparam ShellSpace 	The type of the shell space.
 */
template<typename Path, typename ShellSpace>
std::vector<size_t> tspOnShell( const ApproachPath<Path, ShellSpace>& initial_path,
								const std::vector<Path, ShellSpace>& approach_paths,
								const ShellSpace& shell_space ) {

	std::vector<typename shell_state_traits<ShellSpace>::shell_point_type> shell_points;

	for (const auto& path : approach_paths) {
		shell_points.push_back(shell_state_traits<ShellSpace>::shell_point_for_state(path_traits<Path>::final_state(path), shell_space));
	}

	return tspOnShell(initial_path.shell_point, shell_points, shell_space);

}

/**
 * @brief
 * 	  Given a vector of shell states, and an initial shell state,
 * 	  return a vector of indices of the shell states in the order
 * 	  they should be visited.
 */
template<typename ShellSpace>
std::vector<size_t> tspOnShell( const typename ShellSpace::state_type& initial_shell_state,
								const std::vector<typename ShellSpace::state_type>& shell_states,
								const ShellSpace& shell_space ) {

	// We'll want to invoke our favorite TSP solver on the shell states.
	return tsp_open_end(
			[&](size_t i) {
				return shell_state_traits<ShellSpace>::shell_distance(initial_shell_state, shell_states[i]);
			},
			[&](size_t i, size_t j) {
				return shell_state_traits<ShellSpace>::shell_distance(shell_states[i], shell_states[j]);
			},
			shell_states.size()
			);

}

#endif //NEW_PLANNERS_TSP_ON_SHELL_H
