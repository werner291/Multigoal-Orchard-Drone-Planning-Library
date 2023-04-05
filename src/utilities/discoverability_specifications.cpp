#include <cstddef>
#include <random>
#include <cassert>
#include <algorithm>
#include "discoverability_specifications.h"

std::vector<Proportions> gen_discoverability_proportions() {// Generate a set of probabilities; must sum to 1.

	// FIXME this is bunk, need to fix it if I end up using it again.

	std::vector<Proportions> probs;

	const std::size_t N_PROPORTIONS = 3;

	for (size_t i = 0; i < N_PROPORTIONS; i++) {

		double fraction_true_given = 1.0 - (double) i / (N_PROPORTIONS - 1);

		long sub_proportions = N_PROPORTIONS;

		for (size_t j = 1; j < sub_proportions; j++) {

			double q = (double) j / (double) (sub_proportions - 1);

			probs.emplace_back(Proportions {
					.fraction_true_given = fraction_true_given,
					.fraction_false_given = (1.0 - fraction_true_given) * q,
					.fraction_discoverable = (1.0 - fraction_true_given) * (1.0 - q)
			});
		}
	}
	return probs;
}


std::vector<AppleDiscoverabilityType> generateAppleDiscoverability(Proportions p, int seed, int n_apples) {

	auto rng = std::default_random_engine(seed);

	std::vector<AppleDiscoverabilityType> apple_discoverability(n_apples);

	size_t n_given = ceil(p.fraction_true_given * n_apples);
	size_t n_dynamic = n_apples - n_given;
	size_t n_discoverable = ceil(p.fraction_discoverable * (double) n_dynamic);
	size_t n_false = n_dynamic - n_discoverable;

	size_t i = 0;

	for (size_t ii = 0; ii < n_given; ii++) {
		apple_discoverability[i++] = AppleDiscoverabilityType::GIVEN;
	}

	for (size_t ii = 0; ii < n_discoverable; ii++) {
		apple_discoverability[i++] = AppleDiscoverabilityType::DISCOVERABLE;
	}

	for (size_t ii = 0; ii < n_false; ii++) {
		apple_discoverability[i++] = AppleDiscoverabilityType::FALSE;
	}

	assert(i == n_apples);

	// Shuffle the vector.
	std::shuffle(apple_discoverability.begin(), apple_discoverability.end(), rng);

	return apple_discoverability;

}