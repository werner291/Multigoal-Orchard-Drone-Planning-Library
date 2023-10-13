// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/12/23.
//

#include <gtest/gtest.h>
#include "../src/utilities/math/AABBGrid.h"
#include "../src/utilities/GridVec.h"
#include "../src/voxel_visibility.h"

TEST(VisibitlityTests, visibility_test) {

	using namespace mgodpl::math;

	const size_t GRID_SIZE = 3;

	// Get a 10x10x10 grid.
	AABBGrid grid(Eigen::AlignedBox3d(Eigen::Vector3d(-5.0, -5.0, -5.0), Eigen::Vector3d(5.0, 5.0, 5.0)), GRID_SIZE,GRID_SIZE,GRID_SIZE);

	// Get a dummy opacity grid of all transparent.
	Grid3D<bool> opacity({GRID_SIZE,GRID_SIZE,GRID_SIZE}, false);

	// Generate a random point in the AABB.
//	Eigen::Vector3d point{Eigen::Vector3d::Random() * 5.0};
	// Temporary: all 0:
	Eigen::Vector3d point{0.0, 0.0, 0.0};

	// Compute visibility.
	auto visible = mgodpl::voxel_visibility::opaque_to_visible(grid, opacity, point);

	// Check that the visibility grid is all true.
	for (int x = 0; x < GRID_SIZE; ++x) {
		for (int y = 0; y < GRID_SIZE; ++y) {
			for (int z = 0; z < GRID_SIZE; ++z) {
				ASSERT_TRUE(visible[Eigen::Vector3i(x,y,z)]);
			}
		}
	}
}