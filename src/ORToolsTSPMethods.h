// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 22-2-23.
//

#ifndef NEW_PLANNERS_ORTOOLSTSPMETHODS_H
#define NEW_PLANNERS_ORTOOLSTSPMETHODS_H

#include "IncrementalTSPMethods.h"

/**
 * @brief Determines the optimal position to insert a new goal in a given ordering of goals, such that the sum of
 * distances between consecutive pairs is minimized.
 *
 * @param new_goal The new goal to insert.
 * @param distance A function that takes two goal indices and returns the distance between them.
 * @param first_distance A function that takes a goal index and returns the distance from the start to that goal.
 * @param ordering The current ordering of goals.
 *
 * @return The index at which to insert the new goal in the ordering.
 */
static size_t least_costly_insertion(size_t new_goal,
									 const std::function<double(size_t, size_t)> &distance,
									 const std::function<double(size_t)> &first_distance,
									 const std::vector<size_t> &ordering);

class ORToolsTSPMethods : public IncrementalTSPMethods {

public:
	enum UpdateStrategy {

		LEAST_COSTLY_INSERT, FULL_REORDER

	};

private:
	UpdateStrategy update_strategy;

public:
	explicit ORToolsTSPMethods(UpdateStrategy updateStrategy);

	std::vector<size_t> initial_ordering(size_t n,
										 std::function<double(size_t, size_t)> distance,
										 std::function<double(size_t)> first_distance) override;

	std::vector<size_t> update_ordering(const std::vector<size_t> &current_ordering,
										size_t new_goal,
										std::function<double(size_t, size_t)> distance,
										std::function<double(size_t)> first_distance) override;


};

#endif //NEW_PLANNERS_ORTOOLSTSPMETHODS_H
