
#ifndef NEW_PLANNERS_DISCOVERABILITY_SPECIFICATIONS_H
#define NEW_PLANNERS_DISCOVERABILITY_SPECIFICATIONS_H

#include <vector>
#include <ostream>
#include "math_utils.h"

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

namespace mgodpl {
	namespace utilities {

		/**
		 * @brief Linearly interpolates between two Proportions.
		 *
		 * Given two Proportions and a parameter `t` in the range [0, 1], this function
		 * computes the linear interpolation of the proportions. When `t` is 0, the result
		 * is `a`. When `t` is 1, the result is `b`. For other values of `t`, the result
		 * is a Proportions struct with values linearly interpolated between `a` and `b`.
		 *
		 * @param a The start Proportions.
		 * @param b The end Proportions.
		 * @param t The interpolation parameter, usually in the range [0, 1].
		 * @return The interpolated Proportions between `a` and `b` based on the parameter `t`.
		 */
		template<>
		Proportions lerp(const Proportions &a, const Proportions &b, double t);
	}

	namespace dynamic_goals {

		const Proportions ALL_GIVEN = Proportions {
			.fraction_true_given = 1.0,
			.fraction_false_given = 0.0,
			.fraction_discoverable = 0.0
		};

		const Proportions ALL_FALSE = Proportions {
			.fraction_true_given = 0.0,
			.fraction_false_given = 1.0,
			.fraction_discoverable = 0.0
		};

		const Proportions ALL_DISCOVERABLE = Proportions {
			.fraction_true_given = 0.0,
			.fraction_false_given = 0.0,
			.fraction_discoverable = 1.0
		};

	}
}


/**
 * @brief Generates a set of booleans indicating the discoverability of each apple
 *
 * @param all_apples The number of apples in the scene
 * @return std::vector<bool> A vector of booleans indicating the discoverability of each apple
 */
std::vector<AppleDiscoverabilityType> generateAppleDiscoverability(Proportions p, int seed, int n_apples);

#endif //NEW_PLANNERS_DISCOVERABILITY_SPECIFICATIONS_H
