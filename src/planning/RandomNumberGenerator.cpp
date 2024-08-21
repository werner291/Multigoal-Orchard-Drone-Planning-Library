// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/18/24.
//

#include <algorithm>
#include "RandomNumberGenerator.h"

namespace random_numbers {
	std::vector<size_t> RandomNumberGenerator::pick_indices_without_replacement(size_t n, size_t k) {
		assert(n >= k);

		std::vector<size_t> indices(n);
		std::iota(indices.begin(), indices.end(), 0);

		std::shuffle(indices.begin(), indices.end(), generator);

		indices.resize(k);

		return indices;
	}
} // random_numbers
