
#ifndef NEW_PLANNERS_DISCOVERABILITY_SPECIFICATIONS_H
#define NEW_PLANNERS_DISCOVERABILITY_SPECIFICATIONS_H

#include <vector>

enum AppleDiscoverabilityType {
	GIVEN, DISCOVERABLE, FALSE
};

struct Proportions {
	double fraction_true_given;
	double fraction_false_given;
	double fraction_discoverable;
};

/**
 * @brief Generates a set of booleans indicating the discoverability of each apple
 *
 * @param all_apples The number of apples in the scene
 * @return std::vector<bool> A vector of booleans indicating the discoverability of each apple
 */
std::vector<AppleDiscoverabilityType> generateAppleDiscoverability(int all_apples, Proportions p, int seed, int apples_in_test);

std::vector<Proportions> gen_discoverability_proportions();



#endif //NEW_PLANNERS_DISCOVERABILITY_SPECIFICATIONS_H
