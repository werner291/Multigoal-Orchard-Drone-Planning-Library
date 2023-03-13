
#ifndef NEW_PLANNERS_DISCOVERABILITY_SPECIFICATIONS_H
#define NEW_PLANNERS_DISCOVERABILITY_SPECIFICATIONS_H

#include <vector>
#include <ostream>

enum AppleDiscoverabilityType {
	GIVEN, DISCOVERABLE, FALSE
};

struct Proportions {
	double fraction_true_given;
	double fraction_false_given;
	double fraction_discoverable;

	std::ostream& operator<<(std::ostream& os) const {
		os << "Proportions: " << fraction_true_given << ", " << fraction_false_given << ", " << fraction_discoverable;
		return os;
	}
};

/**
 * @brief Generates a set of booleans indicating the discoverability of each apple
 *
 * @param all_apples The number of apples in the scene
 * @return std::vector<bool> A vector of booleans indicating the discoverability of each apple
 */
std::vector<AppleDiscoverabilityType> generateAppleDiscoverability(Proportions p, int seed, int n_apples);

std::vector<Proportions> gen_discoverability_proportions();



#endif //NEW_PLANNERS_DISCOVERABILITY_SPECIFICATIONS_H
