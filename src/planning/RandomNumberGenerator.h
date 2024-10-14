// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/18/24.
//

#ifndef MGODPL_RANDOMNUMBERGENERATOR_H
#define MGODPL_RANDOMNUMBERGENERATOR_H

#include <random>
#include <cassert>

#include "../math/Vec3.h"

namespace random_numbers {
	/**
	 * Facade to the <random> library.
	 */
	class RandomNumberGenerator {
		// The random number generator.
		std::mt19937 generator;

	public:
		/**
		 * Constructor that takes a seed.
		 */
		RandomNumberGenerator(unsigned int seed) : generator(seed) {
		}

		/**
		 * Constructor that uses a random seed.
		 */
		RandomNumberGenerator() : generator(std::random_device()()) {
		}

		/**
		 * Generate a random number uniformly distributed between 0 and 1.
		 * @return 		A random number uniformly distributed between 0 and 1.
		 */
		inline double uniform01() {
			return std::generate_canonical<double, 10>(generator);
		}

		/**
		 * Generate a random number uniformly distributed between min and max (inclusive).
		 * @param min 		The minimum value.
		 * @param max 		The maximum value (inclusive).
		 * @return 			A random number uniformly distributed between min and max (inclusive).
		 */
		inline int uniformInteger(int min, int max) {
			assert(min <= max);
			std::uniform_int_distribution<int> distribution(min, max);
			return distribution(generator);
		}

		/**
		 * Generate a random number from a canonical Gaussian distribution.
		 */
		inline double gaussian01() {
			std::normal_distribution<double> distribution(0.0, 1.0);
			return distribution(generator);
		}

		inline double uniformReal(double min, double max) {
			std::uniform_real_distribution<double> distribution(min, max);
			return distribution(generator);
		}

		mgodpl::math::Vec3d random_unit_vector() {
			return mgodpl::math::Vec3d(
				gaussian01(),
				gaussian01(),
				gaussian01()
			).normalized();
		}

		/**
		 * Pick k indices without replacement from the range 0..n. (Exclusive of n.)
		 *
		 * If k > n, the function will return a vector of size n with the indices 0..n-1 in random order.
		 *
		 * @param n		The number of indices to pick from.
		 * @param k		The number of indices to pick.
		 * @return		A vector of k indices.
		 */
		std::vector<size_t> pick_indices_without_replacement(size_t n, size_t k);
	};
} // random_numbers

#endif //MGODPL_RANDOMNUMBERGENERATOR_H
