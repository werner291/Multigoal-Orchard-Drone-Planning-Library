// Copyright (c) 2022 University College Roosevelt
// All rights reserved.
#include <random>
#include <stdexcept>

// Includes for range library functionality
#include <range/v3/view/concat.hpp>
#include <range/v3/view/single.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/remove_if.hpp>
#include <range/v3/view/drop.hpp>
#include <range/v3/view/take.hpp>
#include <range/v3/to_container.hpp>

// Include for incremental TSP method utilities
#include "SimpleIncrementalTSPMethods.h"

// Namespace for ranges
using namespace ranges;

// Include for finding min_element
#include <range/v3/algorithm/min_element.hpp>

// Include for iota utility
#include "utilities/size_t_iota.h"


// Function to calculate the cost of inserting a new element at position i
double insertion_cost(size_t i,
					  size_t old_n,
					  std::function<double(const mgodpl::tsp_utils::NewOrderingEntry &,
										   const mgodpl::tsp_utils::NewOrderingEntry &)> distance,
					  const std::function<double(const mgodpl::tsp_utils::NewOrderingEntry &)> &first_distance) {
	using FromOriginal = mgodpl::tsp_utils::FromOriginal;
	using NewGoal = mgodpl::tsp_utils::NewGoal;

	// Make sure old_n is greater than zero
	assert(old_n > 0);

	if (i == 0) {
		// Handle case where we are inserting at the start
		return first_distance(NewGoal{}) +
			   distance(NewGoal{}, FromOriginal{i}) -
			   first_distance(FromOriginal{i});
	} else if (i < old_n) {
		// Handle case where we are inserting in the middle
		return distance(FromOriginal{i - 1}, NewGoal{}) +
			   distance(NewGoal{}, FromOriginal{i}) -
			   distance(FromOriginal{i - 1}, FromOriginal{i});
	} else {
		// Handle case where we are inserting at the end
		return distance(FromOriginal{i - 1}, NewGoal{});
	}
}

// Function to find the least costly insertion point
size_t least_costly_insertion_point(size_t old_n,
									const std::function<double(const mgodpl::tsp_utils::NewOrderingEntry &,
															   const mgodpl::tsp_utils::NewOrderingEntry &)> &distance,
									const std::function<double(const mgodpl::tsp_utils::NewOrderingEntry &)> &first_distance) {

	size_t best_i = 0;
	double best_cost = insertion_cost(0, old_n, distance, first_distance);

	// Iterate over all possible insertion points to find the best one
	for (size_t i = 1; i <= old_n; ++i) {
		double cost = insertion_cost(i, old_n, distance, first_distance);
		if (cost < best_cost) {
			best_i = i;
			best_cost = cost;
		}
	}

	return best_i;
}


namespace mgodpl {
	namespace tsp_utils {

		// Function to insert last
		std::vector<NewOrderingEntry> insert_last(size_t n,
		std::function<double(const NewOrderingEntry &,
							 const NewOrderingEntry &)> distance,
				std::function<double(const NewOrderingEntry &)> first_distance) {

		std::vector<NewOrderingEntry> result;

		for (size_t i = 0; i<n; ++i) {
		result.emplace_back(FromOriginal{i});
	}

	result.emplace_back(NewGoal{});

	return result;
}

// Function to insert first
std::vector<NewOrderingEntry> insert_first(size_t n,
										   std::function<double(const NewOrderingEntry &,
																const NewOrderingEntry &)> distance,
										   std::function<double(const NewOrderingEntry &)> first_distance) {

	std::vector<NewOrderingEntry> result;

	result.emplace_back(NewGoal{});

	for (size_t i = 0; i < n; ++i) {
		result.emplace_back(FromOriginal{i});
	}

	return result;
}

// Function to insert at the second position
std::vector<NewOrderingEntry> insert_second(size_t n,
											std::function<double(const NewOrderingEntry &,
																 const NewOrderingEntry &)> distance,
											std::function<double(const NewOrderingEntry &)> first_distance) {

	std::vector<NewOrderingEntry> result;

	result.emplace_back(FromOriginal{0});

	result.emplace_back(NewGoal{});

	for (size_t i = 1; i < n; ++i) {
		result.emplace_back(FromOriginal{i});
	}

	return result;
}

// Function to insert at the least costly position
std::vector<NewOrderingEntry> insert_least_costly(size_t n,
												  std::function<double(const NewOrderingEntry &,
																	   const NewOrderingEntry &)> distance,
												  std::function<double(const NewOrderingEntry &)> first_distance) {

	std::vector<NewOrderingEntry> result;

	// Find the least costly insertion point
	size_t insert_at = least_costly_insertion_point(n, distance, first_distance);

	// Insert up to the insertion point
	for (size_t i = 0; i < insert_at; ++i) {
		result.emplace_back(FromOriginal{i});
	}

	// Insert the new goal
	result.emplace_back(NewGoal{});

	// Insert the rest
	for (size_t i = insert_at; i < n; ++i) {
		result.emplace_back(FromOriginal{i});
	}

	return result;
}

// Function to insert at a random position
std::vector<NewOrderingEntry> insert_random(size_t n,
											std::function<double(const NewOrderingEntry &,
																 const NewOrderingEntry &)> distance,
											std::function<double(const NewOrderingEntry &)> first_distance) {

	std::vector<NewOrderingEntry> result;

	// Generate a random insertion point
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<size_t> dis(0, n);

	size_t insert_at = dis(gen);

	// Insert up to the insertion point
	for (size_t i = 0; i < insert_at; ++i) {
		result.emplace_back(FromOriginal{i});
	}

	// Insert the new goal
	result.emplace_back(NewGoal{});

	// Insert the rest
	for (size_t i = insert_at; i < n; ++i) {
		result.emplace_back(FromOriginal{i});
	}

	return result;
}
}
}
