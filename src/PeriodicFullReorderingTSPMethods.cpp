// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by Werner Kroneman on 04/08/2023.
//

#include "PeriodicFullReorderingTSPMethods.h"

PeriodicFullReorderingTSPMethods::PeriodicFullReorderingTSPMethods(const std::shared_ptr<IncrementalTSPMethods> &quickMethod,
																   const std::shared_ptr<IncrementalTSPMethods> &slowMethod,
																   const double periodScalingFactor) : quick_method(quickMethod),
																										slow_method(slowMethod),
																										period_scaling_factor(periodScalingFactor),
																										changes_since_last_reordering(0) {
}

std::vector<size_t> PeriodicFullReorderingTSPMethods::initial_ordering(size_t n,
																	   std::function<double(size_t, size_t)> distance,
																	   std::function<double(size_t)> first_distance) {
	return this->quick_method->initial_ordering(n, distance, first_distance);
}

std::vector<IncrementalTSPMethods::NewOrderingEntry> PeriodicFullReorderingTSPMethods::update_ordering_with_insertion(size_t old_n,
																							   std::function<double(
																									   const NewOrderingEntry &,
																									   const NewOrderingEntry &)> distance,
																							   std::function<double(
																									   const NewOrderingEntry &)> first_distance) {
	changes_since_last_reordering += 1;

	if ((double) changes_since_last_reordering > period_scaling_factor * (double) old_n) {
		changes_since_last_reordering = 0;
		return this->slow_method->initial_ordering(old_n, distance, first_distance) | ranges::views::transform([](const auto &entry) {
			if (entry == )
		}) | ranges::to<std::vector>();
	} else {
		return this->quick_method->update_ordering_with_insertion(old_n, distance, first_distance);
	}
}

std::vector<size_t> PeriodicFullReorderingTSPMethods::update_ordering_with_removal(size_t old_n,
																				   size_t removed,
																				   std::function<double(const size_t &,
																										const size_t &)> distance,
																				   std::function<double(const size_t &)> first_distance) {
	return std::vector<size_t>();
}
