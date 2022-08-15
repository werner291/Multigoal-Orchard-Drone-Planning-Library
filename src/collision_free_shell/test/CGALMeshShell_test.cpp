
#include <gtest/gtest.h>

#include "../CGALMeshShell.h"
#include "../CuttingPlaneConvexHullShell.h"
#include "../../utilities/convex_hull.h"
#include "../../utilities/geogebra.h"

TEST(CGALMeshShellTests, test_projection_similarity) {

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

		std::cout << "============ I : " << i << std::endl;
		
		Apple start {
			.center = Eigen::Vector3d(rng.uniformReal(-1, 1), rng.uniformReal(-1, 1), rng.uniformReal(-1, 1)),
			.branch_normal = Eigen::Vector3d(rng.uniformReal(-1, 1), rng.uniformReal(-1, 1), rng.uniformReal(-1, 1)),
		};
		
		Apple end {
			.center = Eigen::Vector3d(rng.uniformReal(-1, 1), rng.uniformReal(-1, 1), rng.uniformReal(-1, 1)),
			.branch_normal = Eigen::Vector3d(rng.uniformReal(-1, 1), rng.uniformReal(-1, 1), rng.uniformReal(-1, 1)),
		};

		auto start_time = std::chrono::high_resolution_clock::now();
		double d1 = shell.predict_path_length(shell.project(start),
											  shell.project(end));
		auto end_time = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

		double d2 = cutting_plane_shell.predict_path_length(cutting_plane_shell.project(start),
															cutting_plane_shell.project(end));

		EXPECT_LE(d1, d2 + 1e-6);
		EXPECT_GE(d1, d2 * 0.9);

	}

}