//
// Created by werner on 9-2-23.
//

#ifndef NEW_PLANNERS_SHELL_SPACE_H
#define NEW_PLANNERS_SHELL_SPACE_H

#include <optional>

template<typename ShellSpace>
struct shell_state_traits {

	// The state type, of which the shell space is a subset.
	using state_type = typename ShellSpace::state_type;

	// An abstract type that designates points in the shell space in a way that is easier to handle internally.
	using shell_point_type = typename ShellSpace::shell_point_type;

	// If a configuration is a shell state. return the corresponding shell point.
	std::optional<state_type> shell_point_for_state(const state_type& state, const ShellSpace& shell) {
		return ShellSpace::shell_point_for_state(state, shell);
	}

	// Compute a shell state for a given shell point
	state_type state_for_shell_point(const shell_point_type& shell_point, const ShellSpace& shell) {
		return ShellSpace::state_for_shell_point(shell_point, shell);
	}

	double shell_distance(const state_type& state1, const state_type& state2, const ShellSpace& shell) {
		return ShellSpace::shell_distance(state1, state2, shell);
	}

	double shell_distance(const shell_point_type& state1, const shell_point_type& state2, const ShellSpace& shell) {
		return ShellSpace::shell_distance(shell_point_for_state(state1, shell), shell_point_for_state(state2, shell), shell);
	}
};

#endif //NEW_PLANNERS_SHELL_SPACE_H
