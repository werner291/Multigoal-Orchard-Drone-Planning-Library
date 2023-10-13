#include <cstddef>
#include <random>
#include <cassert>
#include <algorithm>
#include "discoverability_specifications.h"


std::vector<AppleDiscoverabilityType> generateAppleDiscoverability(Proportions p, int seed, int n_apples) {

	auto rng = std::default_random_engine(seed);

	std::vector<AppleDiscoverabilityType> apple_discoverability(n_apples);

	assert(std::abs(p.fraction_true_given + p.fraction_false_given + p.fraction_discoverable - 1.0) < 1e-6);

	size_t n_given = (size_t) (p.fraction_true_given * n_apples);
	size_t n_dynamic = n_apples - n_given;
	size_t n_discoverable, n_false;

	if (n_dynamic > 0) {
		double fraction_dynamic_discoverable = std::clamp(
				p.fraction_discoverable / (p.fraction_discoverable + p.fraction_false_given), 0.0, 1.0);

		n_discoverable = (size_t) (fraction_dynamic_discoverable * n_dynamic);
		n_false = n_dynamic - n_discoverable;
	} else {
		n_discoverable = 0;
		n_false = 0;
	}

	assert(n_given + n_discoverable + n_false == n_apples);

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

template<>
Proportions mgodpl::utilities::lerp(const Proportions &a, const Proportions &b, double t) {
	Proportions result;

	result.fraction_true_given = lerp(a.fraction_true_given, b.fraction_true_given, t);
	result.fraction_false_given = lerp(a.fraction_false_given, b.fraction_false_given, t);
	result.fraction_discoverable = lerp(a.fraction_discoverable, b.fraction_discoverable, t);

	return result;
}
