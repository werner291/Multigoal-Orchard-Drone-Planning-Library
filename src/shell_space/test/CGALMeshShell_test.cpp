
#include <gtest/gtest.h>

#include "../CGALMeshShell.h"
#include "../CuttingPlaneConvexHullShell.h"
#include "../../utilities/convex_hull.h"
#include "../../utilities/geogebra.h"
#include "../../utilities/experiment_utils.h"


TEST(CGALMeshShellTests, test_prediction_shorter) {

	ompl::RNG rng(44);

	std::vector<geometry_msgs::msg::Point> points;
	points.reserve(100);
	for (int i = 0; i < 100; i++) {
		geometry_msgs::msg::Point p;
		p.x = rng.uniformReal(-1, 1);
		p.y = rng.uniformReal(-1, 1);
		p.z = rng.uniformReal(-1, 1);
		points.push_back(p);
	}

	auto mesh = convexHull(points);

	CGALMeshShell shell(mesh, 0, 0);

	CuttingPlaneConvexHullShell cutting_plane_shell(mesh, 0, 0);

	for (size_t i = 0; i < 100; ++i) {

		Eigen::Vector3d start(rng.uniformReal(-1, 1), rng.uniformReal(-1, 1), rng.uniformReal(-1, 1));
		Eigen::Vector3d goal(rng.uniformReal(-1, 1), rng.uniformReal(-1, 1), rng.uniformReal(-1, 1));

		auto start1 = shell.nearest_point_on_shell(start);
		auto goal1 = shell.nearest_point_on_shell(goal);
		double d1 = shell.path_length(shell.path_from_to(start1, goal1));

		auto start2 = cutting_plane_shell.nearest_point_on_shell(start);
		auto goal2 = cutting_plane_shell.nearest_point_on_shell(goal);
		double d2 = cutting_plane_shell.path_length(cutting_plane_shell.path_from_to(start2, goal2));

		EXPECT_LE(d1, d2 + 1e-6);
		EXPECT_GE(d1, d2 * 0.9);

	}

}
