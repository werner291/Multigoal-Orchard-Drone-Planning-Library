
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

		shape_msgs::msg::Mesh hull = convexHull(points);

		shell = std::make_unique<ConvexHullShell>(hull);

	}

	std::vector<geometry_msgs::msg::Point> points;
	std::unique_ptr<ConvexHullShell> shell;
};


TEST_F(ConvexHullShell_test, all_points_on_or_inside) {

	for (const auto &p : points) {
		EXPECT_TRUE(shell->signed_distance(Eigen::Vector3d(p.x, p.y, p.z)) <= 1.0e-10);
	}



}