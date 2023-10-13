// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/12/23.
//

#include <gtest/gtest.h>
#include <random>
#include "../../src/math/AABBGrid.h"
#include "../../src/experiment_utils/GridVec.h"
#include "../../src/experiment_utils/voxel_visibility.h"
#include "../../src/math/intersections.h"

TEST(VisibilityTests, all_clear) {

	using namespace mgodpl;
	using namespace math;

	const size_t GRID_SIZE = 10;

	// Get a 10x10x10 grid.
	AABBGrid grid({{-5.0, -5.0, -5.0}, {5.0, 5.0, 5.0}}, GRID_SIZE, GRID_SIZE, GRID_SIZE);

	// Get a dummy opacity grid of all transparent.
	Grid3D<bool> opacity({GRID_SIZE,GRID_SIZE,GRID_SIZE}, false);

	// Generate a random point in the AABB.
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(-5.0, 5.0);

	Vec3d point { dis(gen), dis(gen), dis(gen) };

	// Compute visibility.
	auto visible = mgodpl::voxel_visibility::opaque_to_visible(grid, opacity, point, false);

	// Check that the visibility grid is all true.
	for (int x = 0; x < GRID_SIZE; ++x) {
		for (int y = 0; y < GRID_SIZE; ++y) {
			for (int z = 0; z < GRID_SIZE; ++z) {
				ASSERT_TRUE(visible[Vec3i(x,y,z)]);
			}
		}
	}
}

TEST(VisibilityTests, wall) {
	// Import relevant namespaces.
	using namespace mgodpl;
	using namespace math;

	// Define the grid size.
	const size_t GRID_SIZE = 10;

	// Create an axis-aligned bounding box (AABB) encompassing the entire space.
	AABBd total_aabb({{-5.0, -5.0, -5.0}, {5.0, 5.0, 5.0}});

	// Create a 3D grid with the specified size.
	// This grid is used for visibility calculations.
	AABBGrid grid(total_aabb, GRID_SIZE, GRID_SIZE, GRID_SIZE);

	// Create a dummy opacity grid, initially set to all transparent.
	Grid3D<bool> opacity({GRID_SIZE, GRID_SIZE, GRID_SIZE}, false);

	// Define the wall's coordinates within the grid.
	AABBi wall({{4, 4, 4}, {6, 6, 6}});

	// Mark the voxels inside the wall as opaque (not transparent).
	for (int x = wall.min().x(); x < wall.max().x(); ++x) {
		for (int y = wall.min().y(); y < wall.max().y(); ++y) {
			for (int z = wall.min().z(); z < wall.max().z(); ++z) {
				opacity[Vec3i(x, y, z)] = true;
			}
		}
	}

	// Get the real-world coordinates of the wall within the grid.
	auto wall_real = *grid.getAABB(wall);

	// Generate random points in the space between the wall and the edge of the grid.
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(-5.0, 5.0);

	for (int point_rep = 0; point_rep < 100; ++point_rep) {
		Vec3d eye_point(0.0, 0.0, 0.0);

		// Generate random eye_point until it is outside the wall.
		do {
			eye_point = {dis(gen), dis(gen), dis(gen)};
		} while (wall_real.contains(eye_point));

		Vec3d target_point(0.0, 0.0, 0.0);

		// Generate random target_point until it is outside the wall.
		do {
			target_point = {dis(gen), dis(gen), dis(gen)};
		} while (wall_real.contains(target_point));

		// Compute visibility using voxel visibility calculations.
		auto visible = mgodpl::voxel_visibility::opaque_to_visible(grid, opacity, eye_point, false);

		// Check if the actual visibility matches the expected visibility.
		bool actual_visibility = visible[grid.getGridCoordinates(target_point).value()];
		bool expected_visibility = intersects(wall_real, Segment3d(eye_point, target_point));

		// Assert that the actual visibility matches the expected visibility.
		ASSERT_EQ(actual_visibility, expected_visibility);
	}
}
