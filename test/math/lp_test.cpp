// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <gtest/gtest.h>
#include <random>
#include <boost/range/irange.hpp>
#include "../../src/math/Vec3.h"
#include "../../src/math/Plane.h"
#include "../../src/math/lp.h"

TEST(LP, test_find_point) {

	using namespace mgodpl::math;

	// Get a random generator.
	std::mt19937_64 generator(0);

	// Create a random number distribution [ -10.0, 10.0 ]

	std::uniform_real_distribution<double> distribution(-10.0, 10.0);

	for (int i : boost::irange(0,100)) {

		// Generate a random point

		Vec3d pt = {distribution(generator), distribution(generator), distribution(generator)};

		// Generate between 1 and 10 random planes such that the point is inside the half-space defined by the planes.
		int n_planes = std::uniform_int_distribution<int>(1, 10)(generator);

		std::vector<Plane> planes;

		for (int j : boost::irange(0, n_planes)) {

			Vec3d normal = {distribution(generator), distribution(generator), distribution(generator)};
			double rand_scalar = std::uniform_real_distribution<double>(0.0, 10.0)(generator);
			planes.emplace_back(Plane::from_point_and_normal(pt - normal * rand_scalar, normal));

			ASSERT_TRUE(lp_has_solution(planes));

		}

		std::cout << "False cases: " << std::endl;

		for (int j : boost::irange(0, n_planes)) {

			Vec3d normal = {distribution(generator), distribution(generator), distribution(generator)};
			double rand_scalar = std::uniform_real_distribution<double>(-0.01, -10.0)(generator);
			planes.emplace_back(Plane::from_point_and_normal(pt - normal * rand_scalar, normal));

			ASSERT_FALSE(lp_has_solution(planes));

		}

	}


}