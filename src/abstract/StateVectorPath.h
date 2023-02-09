//
// Created by werner on 9-2-23.
//

#ifndef NEW_PLANNERS_STATEVECTORPATH_H
#define NEW_PLANNERS_STATEVECTORPATH_H

#include <vector>
#include "static_shell_approach.h"

template<typename State>
struct StateVectorPath {

	std::vector<State> states;

};

template<typename State>
struct path_traits<StateVectorPath<State>> {
	using state_type = State;

	static StateVectorPath<State> concatenate_path(const StateVectorPath<State> &path1, const StateVectorPath<State> &path2) {
		StateVectorPath<State> new_path;
		new_path.states = path1.states;
		new_path.states.insert(new_path.states.end(), path2.states.begin(), path2.states.end());
		return new_path;
	}

	static state_type initial_state(const StateVectorPath<State> &path) {
		return path.states.front();
	}

	static state_type final_state(const StateVectorPath<State> &path) {
		return path.states.back();
	}
};

#endif //NEW_PLANNERS_STATEVECTORPATH_H
