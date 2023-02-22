// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 22-2-23.
//

#include "ORToolsTSPMethods.h"
#include "utilities/traveling_salesman.h"

std::vector<size_t> ORToolsTSPMethods::initial_ordering(size_t n,
														std::function<double(size_t, size_t)> distance,
														std::function<double(size_t)> first_distance) {

	return tsp_open_end(first_distance, distance, n);

}

std::vector<size_t> ORToolsTSPMethods::update_ordering(const std::vector<size_t> &current_ordering,
													   size_t new_goal,
													   std::function<double(size_t, size_t)> distance,
													   std::function<double(size_t)> first_distance) {

	if (current_ordering.empty()) {
		// If the current ordering is empty, the new ordering is just the new goal
		return {new_goal};
	}

	// Create a new vector that is a copy of the current ordering
	std::vector<size_t> ordering = current_ordering;

	// Insert the new goal at the optimal position in the new ordering
	ordering.insert(ordering.begin() + (long) least_costly_insertion(new_goal, distance, first_distance, ordering),
					new_goal);

	// Return the updated ordering
	return ordering;
}

size_t ORToolsTSPMethods::least_costly_insertion(size_t new_goal,
												 const std::function<double(size_t, size_t)> &distance,
												 const std::function<double(size_t)> &first_distance,
												 const std::vector<size_t> &ordering) {

	// Initialize variables to track the best insertion point and cost
	size_t insert_at = -1; // Position that the new goal will have in the new ordering
	double best_insertion_cost = std::numeric_limits<double>::infinity(); // Minimum cost for inserting the new goal at any position

	// Iterate through the ordering to find the optimal position to insert the new goal
	for (size_t i = 0; i <= ordering.size(); i++) {

		// Calculate the cost of inserting the new goal at this position
		double insertion_cost;

		if (i == 0) {
			// If we are inserting at the start, add the cost of going from the new goal to the first element
			// and subtract the cost of going from the first element to the second element in the current ordering
			insertion_cost = first_distance(new_goal) + distance(new_goal, ordering[i]) - first_distance(ordering[i]);
		} else if (i < ordering.size()) {
			// If we are inserting between two existing elements, add the cost of going from the previous element
			// to the new goal and the cost of going from the new goal to the next element, and subtract the cost of
			// going from the previous element to the next element in the current ordering
			insertion_cost = distance(ordering[i - 1], new_goal) + distance(new_goal, ordering[i]) -
							 distance(ordering[i - 1], ordering[i]);
		} else {
			// If we are inserting at the end, add the cost of going from the previous element to the new goal
			insertion_cost = distance(ordering[i - 1], new_goal);
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

