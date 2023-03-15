// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <stdexcept>
#include "SimpleIncrementalTSPMethods.h"

std::vector<size_t> SimpleIncrementalTSPMethods::initial_ordering(size_t n,
																  std::function<double(size_t, size_t)> distance,
																  std::function<double(size_t)> first_distance) const {

	// Just return the order 0, 1, 2, ..., n-1
	std::vector<size_t> ordering(n);
	for (size_t i = 0; i < n; i++) {
		ordering[i] = i;
	}
	return ordering;

}

std::vector<IncrementalTSPMethods::NewOrderingEntry>
SimpleIncrementalTSPMethods::update_ordering_with_insertion(size_t old_n,
															std::function<double(const NewOrderingEntry &,
																				 const NewOrderingEntry &)> distance,
															std::function<double(const NewOrderingEntry &)> first_distance) const {

	if (old_n == 0) {
		return {NewGoal{}};
	}

	switch (strategy) {
		case LastInFirstOut: {

			// Just return the order 0, 1, 2, ..., n-1
			std::vector<NewOrderingEntry> ordering;

			ordering.emplace_back(NewGoal{});

			for (size_t i = 0; i < old_n; i++) {
				ordering.emplace_back(FromOriginal{i});
			}

			return ordering;

		}

		case FirstInFirstOut: {

			// Just return the order 0, 1, 2, ..., n-1
			std::vector<NewOrderingEntry> ordering;

			for (size_t i = 0; i < old_n; i++) {
				ordering.emplace_back(FromOriginal{i});
			}

			ordering.emplace_back(NewGoal{});

			return ordering;

		}

		case FirstInSecondOut: {

			std::vector<NewOrderingEntry> ordering;

			ordering.emplace_back(FromOriginal{0});

			ordering.emplace_back(NewGoal{});

			for (size_t i = 1; i < old_n; i++) {
				ordering.emplace_back(FromOriginal{i});
			}

			return ordering;
		}
	}


	throw std::runtime_error("Unknown strategy");
}

std::vector<size_t> SimpleIncrementalTSPMethods::update_ordering_with_removal(size_t old_n,
																			  size_t removed,
																			  std::function<double(const size_t &,
																								   const size_t &)> distance,
																			  std::function<double(const size_t &)> first_distance) const {

	// Just delete it.
	std::vector<size_t> ordering(old_n - 1);
	for (size_t i = 0; i < removed; i++) {
		ordering[i] = i;
	}
	for (size_t i = removed + 1; i < old_n; i++) {
		ordering[i - 1] = i;
	}
	return ordering;

}

SimpleIncrementalTSPMethods::SimpleIncrementalTSPMethods(SimpleIncrementalTSPMethods::Strategy strategy) : strategy(
		strategy) {
}
