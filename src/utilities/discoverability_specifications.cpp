#include <cstddef>
#include <random>
#include <cassert>
#include <algorithm>
#include "discoverability_specifications.h"

std::vector<Proportions> gen_discoverability_proportions() {// Generate a set of probabilities; must sum to 1.

	std::vector<Proportions> probs;

	const std::size_t N_PROPORTIONS = 5;

	for (size_t i = 0; i < N_PROPORTIONS; i++) {

		double fraction_true_given = (double) i / (N_PROPORTIONS - 1);

		long sub_proportions = N_PROPORTIONS - i;

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


std::vector<AppleDiscoverabilityType> generateAppleDiscoverability(int all_apples, Proportions p, int seed, int apples_in_test) {

	auto rng = std::default_random_engine(seed);

	std::vector<AppleDiscoverabilityType> apple_discoverability(all_apples);

	size_t n_given = ceil(p.fraction_true_given * apples_in_test);
	size_t n_dynamic = all_apples - n_given;
	size_t n_discoverable = ceil(p.fraction_discoverable * n_dynamic);
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

	assert(i == all_apples);

	// Shuffle the vector.
	std::shuffle(apple_discoverability.begin(), apple_discoverability.end(), rng);

	return apple_discoverability;

}