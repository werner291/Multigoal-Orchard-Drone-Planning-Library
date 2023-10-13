// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/12/23.
//

#include <gtest/gtest.h>
#include <random>
#include <boost/range/irange.hpp>

#include "../../src/math/Vec3.h"
#include "../../src/math/AABBGrid.h"

using namespace mgodpl;
using namespace math;

TEST(AABBGridTest, aabb_test) {

	std::random_device rd;
	std::mt19937 gen(rd());

	for (int rep_i : boost::irange(0,10)) {

		std::uniform_real_distribution<> aabb_dis(-10.0, 10.0);
		AABBd aabb = AABBd::inverted_infinity();
		aabb.expand({aabb_dis(gen), aabb_dis(gen), aabb_dis(gen)});
		aabb.expand({aabb_dis(gen), aabb_dis(gen), aabb_dis(gen)});

		std::uniform_int_distribution<> dis(1, 10);

		size_t nx = dis(gen);
		size_t ny = dis(gen);
		size_t nz = dis(gen);

		mgodpl::math::AABBGrid grid(aabb, nx, ny, nz);

		// For 100 random points...
		for (int pt_i: boost::irange(0, 100)) {

			std::uniform_real_distribution<> dis(0.0, 1.0);

			// Generate a random point in the AABB.
			Vec3d point = aabb.min() + aabb.size() * Vec3d(dis(gen), dis(gen), dis(gen));

			// Get the grid coordinates.
			auto grid_coords = grid.getGridCoordinates(point);

			// Check that the grid coordinates are in range.
			ASSERT_TRUE(grid_coords.has_value());

			// Get the AABB for the grid coordinates.
			auto grid_aabb = grid.getAABB(grid_coords.value());

			// Check that the AABB is in range.
			ASSERT_TRUE(grid_aabb.has_value());

			// Check that the point is in the AABB.
			ASSERT_TRUE(grid_aabb.value().contains(point));

		}
	}
}