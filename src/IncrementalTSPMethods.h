// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 22-2-23.
//

#ifndef NEW_PLANNERS_INCREMENTALTSPMETHODS_H
#define NEW_PLANNERS_INCREMENTALTSPMETHODS_H

#include <vector>
#include <cstddef>
#include <functional>

/**
 * An abstract base class for incremental TSP methods; i.e. methods that can be used to approximately
 * solve the TSP problem, and by providing a way to update the ordering when new goals are added.
 */
class IncrementalTSPMethods {

public:
	virtual std::vector<size_t> initial_ordering(size_t n,
												 std::function<double(size_t, size_t)> distance,
												 std::function<double(size_t)> first_distance) = 0;

	virtual std::vector<size_t> update_ordering(const std::vector<size_t> &current_ordering,
												size_t new_goal,
												std::function<double(size_t, size_t)> distance,
												std::function<double(size_t)> first_distance) = 0;
};


#endif //NEW_PLANNERS_INCREMENTALTSPMETHODS_H
