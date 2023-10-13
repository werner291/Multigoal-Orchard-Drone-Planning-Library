#include <gtest/gtest.h>
#include <random>
#include "../src/shell_space/HorizontalOrientedBoundingBox.h"

class HorizontalOrientedBoundingBoxTest : public ::testing::Test {
protected:
	std::random_device rd;
	std::mt19937 gen;
	std::uniform_real_distribution<> dis;

	HorizontalOrientedBoundingBoxTest() : gen(rd()), dis(-1.0, 1.0) {}

	// Helper function to generate random HOBBPoint
	HOBBPoint random_hobb_point() {

		// Generate a random point:
		Eigen::Vector3d pt(dis(gen), dis(gen), dis(gen));

		// Set a random axis to 1 or -1.

		// Generate a random axis (pick 0,1,2)
		int axis = std::uniform_int_distribution<>(0, 2)(gen);

		// Generate a random sign (pick 0 or 1)
		int sign = std::uniform_int_distribution<>(0, 1)(gen);

		// Set the axis to 1 or -1
		pt[axis] = sign == 0 ? 1.0 : -1.0;

		return HOBBPoint {
			pt
		};
	}

	// Helper function to generate random HorizontalOrientedBoundingBox
	HorizontalOrientedBoundingBox random_hobb() {
//		Eigen::Vector3d center(dis(gen), dis(gen), dis(gen));
//		Eigen::Vector3d half_extents(std::abs(dis(gen)), std::abs(dis(gen)), std::abs(dis(gen)));
//		double rotation = dis(gen);
//		return {center, half_extents, rotation};

		return {

				{ 0.0, 0.0, 0.0 },
				{ 1.0, 1.0, 1.0 },
				0.0

		};
	}
};

TEST_F(HorizontalOrientedBoundingBoxTest, PathFromTo_IsOnSurface) {
	for (int i = 0; i < 100; ++i) {
		HorizontalOrientedBoundingBox hobb = random_hobb();
		HOBBPoint from = random_hobb_point();
		HOBBPoint to = random_hobb_point();

		auto shell_path = hobb.path_from_to(from, to);
		auto path = std::dynamic_pointer_cast<PiecewiseLinearPath<HOBBPoint>>(shell_path);

		for (size_t path_i = 0; path_i < path->points.size() - 1; ++path_i) {

			const auto &point1 = path->points[path_i];
			const auto &point2 = path->points[path_i + 1];

			Eigen::Vector3d surface_point1 = hobb.surface_point(point1);
			Eigen::Vector3d surface_point2 = hobb.surface_point(point2);

			for (int j = 0; j <= 5; ++j) {

				double t = j / 5.0;

				Eigen::Vector3d interpolated_point = surface_point1 * (1 - t) + surface_point2 * t;
				Eigen::Vector3d nearest_point_on_shell = hobb.surface_point(hobb.nearest_point_on_shell(interpolated_point));
				Eigen::Vector3d diff = interpolated_point - nearest_point_on_shell;
				ASSERT_LE(diff.norm(), 1e-6);

			}
		}
	}
}


TEST_F(HorizontalOrientedBoundingBoxTest, PathFromTo_StartsAndEndsCorrectly) {
	for (int i = 0; i < 100; ++i) {
		HorizontalOrientedBoundingBox hobb = random_hobb();
		HOBBPoint from = random_hobb_point();
		HOBBPoint to = random_hobb_point();

		auto shell_path = hobb.path_from_to(from, to);
		auto path = std::dynamic_pointer_cast<PiecewiseLinearPath<HOBBPoint>>(shell_path);

		EXPECT_TRUE((hobb.surface_point(from) -
					 hobb.surface_point(path->points.front())).isApprox(Eigen::Vector3d::Zero(), 1e-6));
		EXPECT_TRUE((hobb.surface_point(to) - hobb.surface_point(path->points.back())).isApprox(Eigen::Vector3d::Zero(),
																								1e-6));
	}
}