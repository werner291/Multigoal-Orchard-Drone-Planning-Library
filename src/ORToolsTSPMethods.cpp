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

std::vector<IncrementalTSPMethods::NewOrderingEntry> ordering_with_insertion(size_t old_n, size_t insert_at) {
	std::vector<IncrementalTSPMethods::NewOrderingEntry> new_ordering(old_n + 1);

	// Copy the elements of the old ordering up to the insertion point
	for (size_t i = 0; i < insert_at; i++) {
		assert(i < new_ordering.size());
		new_ordering[i] = IncrementalTSPMethods::NewOrderingEntry{IncrementalTSPMethods::FromOriginal{i}};
	}

	// Insert the new goal
	assert(insert_at < new_ordering.size());
	new_ordering[insert_at] = IncrementalTSPMethods::NewOrderingEntry{IncrementalTSPMethods::NewGoal{}};

	// Copy the elements of the old ordering after the insertion point
	for (size_t i = insert_at; i < old_n; i++) {
		assert(i < new_ordering.size());
		new_ordering[i + 1] = IncrementalTSPMethods::NewOrderingEntry{IncrementalTSPMethods::FromOriginal{i}};
	}

	return new_ordering;
}


size_t optimal_insertion_point(size_t old_n,
							   const std::function<double(const IncrementalTSPMethods::NewOrderingEntry &,
														  const IncrementalTSPMethods::NewOrderingEntry &)> &distance,
							   const std::function<double(const IncrementalTSPMethods::NewOrderingEntry &)> &first_distance) {

	// Initialize variables to track the best insertion point and cost
	size_t insert_at = -1; // Position that the new goal will have in the new ordering
	double best_insertion_cost = std::numeric_limits<double>::infinity(); // Minimum cost for inserting the new goal at any position

	// Iterate through the ordering to find the optimal position to insert the new goal
	for (size_t i = 0; i <= old_n; i++) {

		// Calculate the cost of inserting the new goal at this position
		double insertion_cost;

		if (i == 0) {
			// If we are inserting at the start, add the cost of going from the new goal to the first element
			// and subtract the cost of going from the first element to the second element in the current ordering
			insertion_cost = first_distance(IncrementalTSPMethods::NewGoal{}) +
							 distance(IncrementalTSPMethods::NewGoal{}, IncrementalTSPMethods::FromOriginal{i}) -
							 first_distance(IncrementalTSPMethods::FromOriginal{i});
		} else if (i < old_n) {
			// If we are inserting between two existing elements, add the cost of going from the previous element
			// to the new goal and the cost of going from the new goal to the next element, and subtract the cost of
			// going from the previous element to the next element in the current ordering
			insertion_cost = distance(IncrementalTSPMethods::FromOriginal{i - 1}, IncrementalTSPMethods::NewGoal{}) +
							 distance(IncrementalTSPMethods::NewGoal{}, IncrementalTSPMethods::FromOriginal{i}) -
							 distance(IncrementalTSPMethods::FromOriginal{i - 1},
									  IncrementalTSPMethods::FromOriginal{i});
		} else {
			// If we are inserting at the end, add the cost of going from the previous element to the new goal
			insertion_cost = distance(IncrementalTSPMethods::FromOriginal{i - 1}, IncrementalTSPMethods::NewGoal{});
		}

		// If this insertion point has a lower cost than the previous best insertion point, update the best cost
		// and insertion point
		if (insertion_cost < best_insertion_cost) {
			best_insertion_cost = insertion_cost;
			insert_at = i;
		}
	}

	return insert_at;
}

std::vector<IncrementalTSPMethods::NewOrderingEntry> ORToolsTSPMethods::update_ordering(size_t old_n,
																						std::function<double(const NewOrderingEntry &,
																											 const NewOrderingEntry &)> distance,
																						std::function<double(const NewOrderingEntry &)> first_distance) const {

	// Change the order based on the given update strategy.
	switch (update_strategy) {
		// LEAST_COSTLY_INSERT: Greedily insert the new goal at the position that minimizes the cost of the new ordering.
		case LEAST_COSTLY_INSERT: {

			// If the ordering is empty, just add the new goal
			if (old_n == 0) {
				return {NewOrderingEntry{NewGoal{}}};
			}

			// Otherwise, find the optimal insertion point and insert the new goal there
			return ordering_with_insertion(old_n, optimal_insertion_point(old_n, distance, first_distance));

		}
			// FULL_REORDER: Reorder the entire ordering, including the new goal, from scratch.
		case FULL_REORDER: {

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
		default:
			throw std::runtime_error("Invalid update strategy");
	}


}

ORToolsTSPMethods::ORToolsTSPMethods(ORToolsTSPMethods::UpdateStrategy updateStrategy)
		: update_strategy(updateStrategy) {
}
