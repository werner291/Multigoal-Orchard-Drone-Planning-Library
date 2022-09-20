
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


TEST(CGALMeshShellTests, test_path_shorter) {

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

	auto drone = loadRobotModel();

	rclcpp::init(0, nullptr);
	auto evt = std::make_shared<ExperimentVisualTools>();

	evt->publishMesh(mesh, "hull");
	rclcpp::spin_some(evt);


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

		auto p1 = shell.path_on_shell(drone, shell.project(start), shell.project(end));
		double d_cgal = RobotPath{p1}.length();

		auto p2 = cutting_plane_shell.path_on_shell(drone, cutting_plane_shell.project(start), cutting_plane_shell.project(end));
		double d_cutplane = RobotPath{p2}.length();

		if (d_cgal > d_cutplane * 1.1) {

			evt->publishPath("cgalpath", RobotPath{p1});
			evt->publishPath("cutplanepath", RobotPath{p2});
			rclcpp::spin(evt);
		}

		EXPECT_LE(d_cgal, d_cutplane * 1.1);
//		EXPECT_GE(de1, d_cutplane * 0.9);

	}

}