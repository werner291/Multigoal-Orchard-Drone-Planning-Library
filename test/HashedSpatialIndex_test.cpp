
#include <gtest/gtest.h>
#include <ompl/util/RandomNumbers.h>
#include <Eigen/Core>
#include "../src/HashedSpatialIndex.h"

TEST(HashedSpatialIndex_test, find_back_noisy_copy) {

	// Generate 100 random points in a [10,10,10] box.

	ompl::RNG rng(43);

	std::vector<Eigen::Vector3d> points;
	points.reserve(1000);

	for (int i = 0; i < 1000; i++) {
		points.emplace_back(rng.uniformReal(-5, 5), rng.uniformReal(-5, 5), rng.uniformReal(-5, 5));
	}

	// Create a HashedSpatialIndex with a 0.5 resolution.
	HashedSpatialIndex<std::monostate> index(0.5, 100);

	for (const auto &point : points) {
		index.insert(point, {});
	}

	// Add noise of norm at most 0.5 to all the points.
	for (auto &point : points) {

		Eigen::Vector3d noise(rng.gaussian(0, 1), rng.gaussian(0, 1), rng.gaussian(0, 1));
		noise.normalize();
		noise *= rng.uniformReal(0, 0.5);

		point += noise;

	}

	// Check that all points are found.
	for (const auto &point : points) {
		EXPECT_TRUE(index.any_within(point, 0.5));
	}
}