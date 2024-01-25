// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/22/24.
//

#include <cstddef>
#include <vector>
#include <limits>
#include "visitation_order.h"

std::vector<size_t> mgodpl::visitation_order_greedy(const std::vector<std::vector<double>> &target_to_target_distances,
													const std::vector<double> &initial_state_distances) {
	std::vector<bool> used(target_to_target_distances.size(), false);

	std::vector<size_t> order;
	order.reserve(target_to_target_distances.size());
	for (size_t i = 0; i < target_to_target_distances.size(); ++i) {

		// First one uses initial_state_distances; the rest uses target_to_target_distances.
		const auto &distances = i == 0 ? initial_state_distances : target_to_target_distances[order.back()];

		// Find the closest one that hasn't been used yet.
		size_t closest = 0;
		double closest_distance = std::numeric_limits<double>::infinity();
		for (size_t j = 0; j < distances.size(); ++j) {
			if (!used[j] && distances[j] < closest_distance) {
				closest = j;
				closest_distance = distances[j];
			}
		}

		// Mark it as used.
		used[closest] = true;
		order.push_back(closest);
	}
	return order;
}
