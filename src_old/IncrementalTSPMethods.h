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

namespace mgodpl {
	namespace tsp_utils {

		/**
		 * A function type for computing an initial ordering of the goals.
		 */
		using InitialOrderingFunc = std::function<std::vector<size_t>(size_t,
																	  std::function<double(size_t, size_t)>,
																	  std::function<double(size_t)>)>;

		/**
		 * An index into the original vector.
		 */
		struct FromOriginal {
			size_t index;
		};

		/**
		 * The new goal, understood from context.
		 */
		struct NewGoal {};

		/**
		 * A variant referring to either an index into the original vector, or a new goal.
		 */
		using NewOrderingEntry = std::variant<FromOriginal, NewGoal>;

		/**
	 * Return a new ordering of the goals, assuming the addition of a new goal.
	 *
	 * @param old_n 			The number of goals in the original ordering.
	 * @param distance 			A distance function that takes two NewOrderingEntry objects and returns the distance between them.
	 * @param first_distance 	A distance function that takes a NewOrderingEntry object and returns the distance from the start to it.
	 * @return 					A vector of NewOrderingEntry objects representing the new ordering.
	 */
		using InsertionOrderingFunc = std::function<std::vector<NewOrderingEntry>(size_t,
																				  std::function<double(const NewOrderingEntry &,
																									   const NewOrderingEntry &)>,
																				  std::function<double(const NewOrderingEntry &)>)>;

		/**
	 * Return a new ordering of the goals, assuming the removal of a goal.
	 *
	 * @param old_n 			The number of goals in the original ordering.
	 * @param removed 			The index of the removed goal.
	 * @param distance 			A distance function that takes two NewOrderingEntry objects and returns the distance between them.
	 * @param first_distance 	A distance function that takes a NewOrderingEntry object and returns the distance from the start to it.
	 */
		using RemovalOrderingFunc = std::function<std::vector<size_t>(size_t,
																	  size_t,
																	  std::function<double(const size_t &,
																						   const size_t &)>,
																	  std::function<double(const size_t &)>)>;

		/**
		 * A struct holding function pointers for the update methods of the TSP.
		 */
		struct UpdateTSPMethods {
			InsertionOrderingFunc update_ordering_with_insertion;
			RemovalOrderingFunc update_ordering_with_removal;
		};

		/**
		 * A struct holding function pointers for both the initial and update methods of the TSP.
		 */
		struct IncrementalTSPMethods {
			InitialOrderingFunc initial_ordering;
			UpdateTSPMethods update_methods;
		};

		/**
		 * A function that wraps an InitialOrderingFunc to provide InsertionOrderingFunc functionality.
		 */
		InsertionOrderingFunc insertionByFullReordering(InitialOrderingFunc initial_ordering);

		/**
		 * A function that wraps an InitialOrderingFunc to provide RemovalOrderingFunc functionality.
		 */
		RemovalOrderingFunc removalByFullReordering(InitialOrderingFunc initial_ordering);

		/**
		 * Quick convenience function to wrap up insertionByFullReordering and removalByFullReordering.
		 */
		UpdateTSPMethods fullReordering(InitialOrderingFunc initial_ordering);

		/**
		 * Convenience function that adapts a InitialOrderingFunc to an IncrementalTSPMethods.
		 * @param initial_ordering 		The InitialOrderingFunc to adapt.
		 * @return 						An IncrementalTSPMethods that uses the given InitialOrderingFunc.
		 */
		IncrementalTSPMethods incrementalTspFromSimpleOrderngTSP(InitialOrderingFunc initial_ordering);



		/**
		 * A RemovalOrderingFunc that simply deletes the item, preserving the order of the other items.
		 */
		std::vector<size_t> removalBySimpleDeletion(size_t old_n, size_t removed,
											  std::function<double(const size_t &,
																   const size_t &)> distance,
											  std::function<double(const size_t &)> first_distance);

	}  // namespace tsp_utils
}  // namespace mgodpl



#endif //NEW_PLANNERS_INCREMENTALTSPMETHODS_H
