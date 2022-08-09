
#include <gtest/gtest.h>
#include "../ConvexHullShell.h"
#include "../../utilities/convex_hull.h"

class ConvexHullShell_test : public ::testing::Test {
protected:
	void SetUp() override {

		// Generate a random set of 100 points
		ompl::RNG rng;

		points.reserve(100);

		for (int i = 0; i < 100; i++) {
			geometry_msgs::msg::Point p;
			p.x = rng.uniformReal(-1, 1);
			p.y = rng.uniformReal(-1, 1);
			p.z = rng.uniformReal(-1, 1);
			points.push_back(p);
		}

		shell = std::make_unique<ConvexHullShell>(convexHull(points));

		std::vector<geometry_msgs::msg::Point> cube_points;
		cube_points.reserve(8);
		for (int i = 0; i < 8; i++) {
			geometry_msgs::msg::Point p;
			p.x = i & 1 ? 1 : -1;
			p.y = i & 2 ? 1 : -1;
			p.z = i & 4 ? 1 : -1;
			cube_points.push_back(p);
		}

		cube_shell = std::make_unique<ConvexHullShell>(convexHull(cube_points));


	}

	std::vector<geometry_msgs::msg::Point> points;
	std::unique_ptr<ConvexHullShell> shell;

	std::unique_ptr<ConvexHullShell> cube_shell;
};


TEST_F(ConvexHullShell_test, all_points_on_or_inside) {

	for (const auto &p : points) {
		EXPECT_TRUE(shell->signed_distance(Eigen::Vector3d(p.x, p.y, p.z)) <= 1.0e-10);
	}

}

TEST_F(ConvexHullShell_test, cube_projections) {

	ompl::RNG rng(42);

	for (size_t i = 0; i < 1000; ++i) {
		Eigen::Vector3d p;

		p.x() = rng.uniformReal(-2.0, 2.0);
		p.y() = rng.uniformReal(-2.0, 2.0);
		p.z() = rng.uniformReal(-2.0, 2.0);

		auto proj = cube_shell->project(Apple {p, {0.0, 0.0, 0.0}});

		Eigen::Vector3d expected_proj {
			std::clamp(p.x(), -1.0, 1.0),
			std::clamp(p.y(), -1.0, 1.0),
			std::clamp(p.z(), -1.0, 1.0)
		};

		std::cout << "p: " << p.transpose() << std::endl;
		std::cout << "proj: " << proj.position.transpose() << std::endl;
		std::cout << "expected_proj: " << expected_proj.transpose() << std::endl;

		ASSERT_NEAR(expected_proj.x(), proj.position.x(), 1.0e-10);
		ASSERT_NEAR(expected_proj.y(), proj.position.y(), 1.0e-10);
		ASSERT_NEAR(expected_proj.z(), proj.position.z(), 1.0e-10);
	}

}