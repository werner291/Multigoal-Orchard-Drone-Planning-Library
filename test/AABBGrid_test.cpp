// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/12/23.
//

#include <gtest/gtest.h>
#include <random>
#include <boost/range/irange.hpp>

#include "../src/utilities/math/AABBGrid.h"

TEST(AABBGridTest, aabb_test) {

	for (int rep_i : boost::irange(0,10)) {
		// Generate a random AABB.
		Eigen::AlignedBox3d aabb(Eigen::Vector3d::Random() * 100.0, Eigen::Vector3d::Random() * 100.0);

		// Swap the min and max if necessary.
		for (int i = 0; i < 3; ++i) {
			if (aabb.min()[i] > aabb.max()[i]) {
				std::swap(aabb.min()[i], aabb.max()[i]);
			}
		}

		// Pick random subdivisions between 1 and 10.
		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_int_distribution<> dis(1, 10);

		size_t nx = dis(gen);
		size_t ny = dis(gen);
		size_t nz = dis(gen);

		mgodpl::math::AABBGrid grid(aabb, nx, ny, nz);

		// For 100 random points...
		for (int pt_i: boost::irange(0, 100)) {

			std::uniform_real_distribution<> dis(0.0, 1.0);

			// Generate a random point in the AABB.
			Eigen::Vector3d point{aabb.min().x() + dis(gen) * aabb.sizes().x(),
								  aabb.min().y() + dis(gen) * aabb.sizes().y(),
								  aabb.min().z() + dis(gen) * aabb.sizes().z()};

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