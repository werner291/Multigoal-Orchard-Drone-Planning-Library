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
#include <variant>

/**
 * An abstract base class for incremental TSP methods; i.e. methods that can be used to approximately
 * solve the TSP problem, and by providing a way to update the ordering when new goals are added.
 */
class IncrementalTSPMethods {

public:
	/**
	 * @brief Compute an initial ordering of the goals.
	 *
	 * Goals are treated merely as indices.
	 *
	 * @param n 				The number of goals.
	 * @param distance 			A function that returns the distance between two goals (represented by their indices).
	 * @param first_distance 	A function that returns the distance from the start to a goal (represented by its index).
	 * @return 					A vector of indices representing the initial ordering.
	 */
	virtual std::vector<size_t> initial_ordering(size_t n,
												 std::function<double(size_t, size_t)> distance,
												 std::function<double(size_t)> first_distance) = 0;

	/**
	 * An index into the original vector.
	 */
	struct FromOriginal {
		size_t index;
	};

	/**
	 * The new goal, understood from context.
	 */
	struct NewGoal {
	};

	/**
	 * A variant referring to either an index into the original vector, or a new goal.
	 */
	using NewOrderingEntry = std::variant<FromOriginal, NewGoal>;

	/**
	 * Return a new ordering of the goals.
	 *
	 * @param old_n 			The number of goals in the original ordering.
	 * @param distance 			A distance function that takes two NewOrderingEntry objects and returns the distance between them.
	 * @param first_distance 	A distance function that takes a NewOrderingEntry object and returns the distance from the start to it.
	 * @return 					A vector of NewOrderingEntry objects representing the new ordering.
	 */
	virtual std::vector<NewOrderingEntry> update_ordering(size_t old_n,
														  std::function<double(const NewOrderingEntry &,
																			   const NewOrderingEntry &)> distance,
														  std::function<double(const NewOrderingEntry &)> first_distance) = 0;
};


#endif //NEW_PLANNERS_INCREMENTALTSPMETHODS_H
