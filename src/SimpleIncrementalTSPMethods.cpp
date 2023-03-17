// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.
#include <random>
#include <stdexcept>

#include <range/v3/view/concat.hpp>
#include <range/v3/view/single.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/remove_if.hpp>
#include <range/v3/view/drop.hpp>
#include <range/v3/view/take.hpp>

#include <range/v3/to_container.hpp>

#include "SimpleIncrementalTSPMethods.h"

using namespace ranges;

#include <range/v3/algorithm/min_element.hpp>

#include "utilities/size_t_iota.h"

// Function to calculate the cost of inserting a new element at position i.
// The cost is based on the change in the sum of distances between consecutive elements.
double insertion_cost(size_t i,
					  size_t old_n,
					  std::function<double(const IncrementalTSPMethods::NewOrderingEntry &,
										   const IncrementalTSPMethods::NewOrderingEntry &)> distance,
					  const std::function<double(const IncrementalTSPMethods::NewOrderingEntry &)> &first_distance) {
	using FromOriginal = IncrementalTSPMethods::FromOriginal;
	using NewGoal = IncrementalTSPMethods::NewGoal;

	assert(old_n > 0);

	// If the original ordering is (start)-0-1-2-3-4-5-6-7-8-9,

	if (i == 0) {

		// We get the ordering (start)-(new goal)-0-1-2-3-4-5-6-7-8-9 by inserting at the start.
		// Inserting at the start would add the segments: (start)-(new goal) and (new goal)-0, and delete the segment (start)-0.

		return first_distance(NewGoal{}) + // (start)-(new goal)
			   distance(NewGoal{}, FromOriginal{i}) - // (new goal)-0
			   first_distance(FromOriginal{i}); // The deleted segment (start)-0
	} else if (i < old_n) {

		// We get the ordering (start)-0-1-2-3-(new goal)-4-5-6-7-8-9 by inserting at position 4 (if i == 4).
		// Inserting at position 4 would add the segments: 3-(new goal) and (new goal)-4, and delete the segment 3-4.

		return distance(FromOriginal{i - 1}, NewGoal{}) + // 3-(new goal)
			   distance(NewGoal{}, FromOriginal{i}) - // (new goal)-4
			   distance(FromOriginal{i - 1}, FromOriginal{i}); // The deleted segment 3-4
	} else {

		// We get the ordering (start)-0-1-2-3-4-5-6-7-8-9-(new goal) by inserting at the end.
		// Inserting at the end would add the segments: 9-(new goal).

		return distance(FromOriginal{i - 1}, NewGoal{}); // 9-(new goal)
	}
}

size_t least_costly_insertion_point(size_t old_n,
									const std::function<double(const IncrementalTSPMethods::NewOrderingEntry &,
															   const IncrementalTSPMethods::NewOrderingEntry &)> &distance,
									const std::function<double(const IncrementalTSPMethods::NewOrderingEntry &)> &first_distance) {

	size_t best_i = 0;
	double best_cost = insertion_cost(0, old_n, distance, first_distance);

	for (size_t i = 1; i <= old_n; ++i) {
		double cost = insertion_cost(i, old_n, distance, first_distance);
		if (cost < best_cost) {
			best_i = i;
			best_cost = cost;
		}
	}

	return best_i;
}


// Computes the initial ordering for the given input.
std::vector<size_t> SimpleIncrementalTSPMethods::initial_ordering(size_t n,
																  std::function<double(size_t, size_t)> distance,
																  std::function<double(size_t)> first_distance) const {

	switch (strategy) { // Operate depending on the strategy.

		case LastInFirstOut: {
			// The ordering is the reverse of the original ordering.
			// This choice is a bit arbitrary, but it'd be consistent with adding the given points one-by-one according to this strategy.
			std::vector<size_t> ordering = size_t_iota(0, n) | ranges::to<std::vector>;
			std::reverse(ordering.begin(), ordering.end());
			return ordering;
		}
			break;
		case FirstInFirstOut: // Just use the original ordering.
		case FirstInSecondOut: // Just use the original ordering as well, since this strategy is just there because some planners react a bit different if updates, specifically, are either the first or the second element
			return size_t_iota(0, n) | ranges::to<std::vector>;
			break;
		case LeastCostlyInsertion: {
			// The ordering is built element-by-element by inserting the new element at the position that minimizes the cost of insertion.

			std::vector<size_t> ordering{0}; // Start with the first element.

			for (size_t i = 1; i < n; ++i) { // For each new element...

				// We need to translate indices, because the given distance and first_distance functions are based
				// on the original ordering, but we're building the new ordering element-by-element.
				auto translate_index = [&](NewOrderingEntry entry) {
					if (auto *from_original = std::get_if<FromOriginal>(&entry)) {
						assert(from_original->index < i);
						return ordering[from_original->index];
					} else {
						return i;
					}
				};

				size_t insert_point = least_costly_insertion_point(ordering.size(),
																   [&](const NewOrderingEntry &a,
																	   const NewOrderingEntry &b) {
																	   return distance(translate_index(a),
																					   translate_index(b));
																   },
																   [&](const NewOrderingEntry &a) {
																	   return first_distance(translate_index(a));
																   });

				ordering.insert(ordering.begin() + (long) insert_point, i);

			}

			return ordering;

		}
			break;
		case Random:
			std::vector<size_t> ordering = size_t_iota(0, n) | ranges::to<std::vector>;
			std::shuffle(ordering.begin(), ordering.end(), std::mt19937(std::random_device()()));
			return ordering;
			break;
	}

	throw std::logic_error("Strategy not implemented");

}



// Updates the ordering after a new element is inserted.
std::vector<IncrementalTSPMethods::NewOrderingEntry>
SimpleIncrementalTSPMethods::update_ordering_with_insertion(size_t old_n,
															std::function<double(const NewOrderingEntry &,
																				 const NewOrderingEntry &)> distance,
															std::function<double(const NewOrderingEntry &)> first_distance) const {
	if (old_n == 0) {
		return {NewGoal{}};
	}

	auto from_original_range = size_t_iota(0, old_n) | views::transform([](size_t i) { return NewOrderingEntry{FromOriginal{i}}; });

	size_t insert_at = insertionPointByStrategy(old_n, distance, first_distance);

	return ranges::to<std::vector>(views::concat(from_original_range | views::take(insert_at),
												 views::single(NewOrderingEntry{NewGoal{}}),
												 from_original_range | views::drop(insert_at)));
}

size_t SimpleIncrementalTSPMethods::insertionPointByStrategy(size_t old_n,
															 const std::function<double(const NewOrderingEntry &,
																						const NewOrderingEntry &)> &distance,
															 const std::function<double(const NewOrderingEntry &)> &first_distance) const {
	size_t insert_at;

	switch (strategy) {
		case LastInFirstOut:
			insert_at = 0;
			break;
		case FirstInFirstOut:
			insert_at = old_n;
			break;
		case FirstInSecondOut:
			insert_at = 1;
			break;
		case LeastCostlyInsertion:
			insert_at = least_costly_insertion_point(old_n, distance, first_distance);
			break;
		case Random: {
			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_int_distribution<size_t> dis(0, old_n);
			insert_at = dis(gen);
		}
			break;

		default:
			throw std::runtime_error("Unknown strategy");
	}
	return insert_at;
}


// Updates the ordering after an element is removed.
std::vector<size_t> SimpleIncrementalTSPMethods::update_ordering_with_removal(size_t old_n,
																			  size_t removed,
																			  std::function<double(const size_t &,
																								   const size_t &)> distance,
																			  std::function<double(const size_t &)> first_distance) const {
	return size_t_iota(0, old_n) | views::remove_if([=](size_t i) { return i == removed; }) | to<std::vector>();
}

// Constructor for SimpleIncrementalTSPMethods.
SimpleIncrementalTSPMethods::SimpleIncrementalTSPMethods(SimpleIncrementalTSPMethods::Strategy strategy) : strategy(strategy) {
}
