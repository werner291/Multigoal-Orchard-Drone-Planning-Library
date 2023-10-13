// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 22-2-23.
//

#include "IncrementalTSPMethods.h"
#include "utilities/size_t_iota.h"

#include <range/v3/view/transform.hpp>
#include <range/v3/to_container.hpp>
#include <range/v3/view/remove_if.hpp>

mgodpl::tsp_utils::InsertionOrderingFunc
mgodpl::tsp_utils::insertionByFullReordering(mgodpl::tsp_utils::InitialOrderingFunc initial_ordering) {
	return [initial_ordering](size_t old_n,
							  std::function<double(const NewOrderingEntry &,
												   const NewOrderingEntry &)> distance,
							  std::function<double(const NewOrderingEntry &)> first_distance) {
		// Invoke the initial ordering function to get a new ordering.
		// Since initial_ordering only works in terms of indices, by convention, old_n shall be the index of the new goal
		auto new_ordering = initial_ordering(old_n + 1, [&](size_t i, size_t j) -> double {
			return distance(i < old_n ? NewOrderingEntry{FromOriginal{i}} : NewOrderingEntry{NewGoal{}},
							j < old_n ? NewOrderingEntry{FromOriginal{j}} : NewOrderingEntry{NewGoal{}});
		}, [&](size_t i) -> double {
			return first_distance(i < old_n ? NewOrderingEntry{FromOriginal{i}} : NewOrderingEntry{NewGoal{}});
		});

		// Return a vector of indices, using the variant to distinguish between the new goal and the original elements
		return new_ordering | ranges::views::transform([&](size_t i) -> NewOrderingEntry {
			return i < old_n ? NewOrderingEntry{FromOriginal{i}} : NewOrderingEntry{NewGoal{}};
		}) | ranges::to_vector;
	};
}

mgodpl::tsp_utils::RemovalOrderingFunc
mgodpl::tsp_utils::removalByFullReordering(mgodpl::tsp_utils::InitialOrderingFunc initial_ordering) {
	return [initial_ordering](size_t old_n,
							  size_t removed,
							  std::function<double(const size_t &,
												   const size_t &)> distance,
							  std::function<double(const size_t &)> first_distance) {
		// Invoke the initial ordering function to get a new ordering.
		auto new_ordering = initial_ordering(old_n - 1, [&](size_t i, size_t j) -> double {
			// If either of the indices is the removed element, we need to skip it,
			// so we add 1 to the index if it is greater than the removed element
			return distance(i < removed ? i : i + 1, j < removed ? j : j + 1);
		}, [&](size_t i) -> double {
			return first_distance(i < removed ? i : i + 1);
		});

		// Return a vector of indices, adjusting the indices to account for the removed element
		return new_ordering | ranges::views::transform([&](size_t i) -> size_t {
			return i < removed ? i : i + 1;
		}) | ranges::to_vector;
	};
}

mgodpl::tsp_utils::UpdateTSPMethods
mgodpl::tsp_utils::fullReordering(mgodpl::tsp_utils::InitialOrderingFunc initial_ordering) {
	return UpdateTSPMethods{insertionByFullReordering(initial_ordering),
							removalByFullReordering(initial_ordering)};
}


/**
		 * A RemovalOrderingFunc that simply deletes the item, preserving the order of the other items.
		 */
std::vector<size_t> mgodpl::tsp_utils::removalBySimpleDeletion(size_t old_n, size_t removed,
															   std::function<double(const size_t &,const size_t &)> distance,
															   std::function<double(const size_t &)> first_distance) {
	return size_t_iota(0, old_n) | ranges::views::remove_if([=](size_t i) { return i == removed; }) | ranges::to<std::vector>();
}

mgodpl::tsp_utils::IncrementalTSPMethods
mgodpl::tsp_utils::incrementalTspFromSimpleOrderngTSP(mgodpl::tsp_utils::InitialOrderingFunc initial_ordering) {
	return  {
			initial_ordering,
			mgodpl::tsp_utils::fullReordering(initial_ordering)
	};
}
