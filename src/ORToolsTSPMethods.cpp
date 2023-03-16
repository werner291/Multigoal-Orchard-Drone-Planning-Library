// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 22-2-23.
//

#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include "ORToolsTSPMethods.h"
#include "utilities/traveling_salesman.h"

std::vector<size_t> ORToolsTSPMethods::initial_ordering(size_t n,
														std::function<double(size_t, size_t)> distance,
														std::function<double(size_t)> first_distance) const {

	return tsp_open_end(first_distance, distance, n);

}

std::vector<IncrementalTSPMethods::NewOrderingEntry> ORToolsTSPMethods::update_ordering_with_insertion(size_t old_n,
																									   std::function<double(
																											   const NewOrderingEntry &,
																											   const NewOrderingEntry &)> distance,
																									   std::function<double(
																											   const NewOrderingEntry &)> first_distance) const {

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
}

ORToolsTSPMethods::ORToolsTSPMethods() {
}

std::vector<size_t> ORToolsTSPMethods::update_ordering_with_removal(size_t old_n,
																	size_t removed,
																	std::function<double(const size_t &,
																						 const size_t &)> distance,
																	std::function<double(const size_t &)> first_distance) const {

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

}
