// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.
#include <stdexcept>

#include <range/v3/view/concat.hpp>
#include <range/v3/view/single.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/remove_if.hpp>
#include <range/v3/view/drop.hpp>
#include <range/v3/view/take.hpp>

#include <range/v3/to_container.hpp>

#include "SimpleIncrementalTSPMethods.h"

using namespace ranges;

#include <range/v3/view_facade.hpp>
#include <range/v3/algorithm/min_element.hpp>

#include "utilities/size_t_iota.h"


// Computes the initial ordering for the given input.
std::vector<size_t> SimpleIncrementalTSPMethods::initial_ordering(size_t n,
																  std::function<double(size_t, size_t)> distance,
																  std::function<double(size_t)> first_distance) const {
	return size_t_iota(0, n) | to<std::vector>();
}

size_t least_costly_insertion_point(size_t old_n,
									const std::function<double(const IncrementalTSPMethods::NewOrderingEntry &,
															   const IncrementalTSPMethods::NewOrderingEntry &)> &distance,
									const std::function<double(const IncrementalTSPMethods::NewOrderingEntry &)> &first_distance) {

	using FromOriginal = IncrementalTSPMethods::FromOriginal;
	using NewGoal = IncrementalTSPMethods::NewGoal;

	assert(old_n > 0);

	auto insertion_cost = [&](size_t i) {
		if (i == 0) {
			return first_distance(NewGoal{}) +
				   distance(NewGoal{}, FromOriginal{i}) -
				   first_distance(FromOriginal{i});
		} else if (i < old_n) {
			return distance(FromOriginal{i - 1}, NewGoal{}) +
				   distance(NewGoal{}, FromOriginal{i}) -
				   distance(FromOriginal{i - 1},
							FromOriginal{i});
		} else {
			return distance(FromOriginal{i - 1}, NewGoal{});
		}
	};

	return ranges::min(size_t_iota(0, old_n + 1), ranges::less{}, insertion_cost);
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
		default:
			throw std::runtime_error("Unknown strategy");
	}

	return ranges::to<std::vector>(views::concat(from_original_range | views::take(insert_at),
												 views::single(NewOrderingEntry{NewGoal{}}),
												 from_original_range | views::drop(insert_at)));
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
