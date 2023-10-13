
#include <gtest/gtest.h>

#include "../src/shell_space/CGALMeshShell.h"
#include "../src/shell_space/CuttingPlaneConvexHullShell.h"
#include "../src/utilities/convex_hull.h"
#include "../src/utilities/geogebra.h"
#include "../src/utilities/experiment_utils.h"

shape_msgs::msg::Mesh mkConvexMesh() {
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

	return mesh;
}

TEST(CGALMeshShellTests, test_prediction_shorter) {

	ompl::RNG rng(44);


	auto mesh = mkConvexMesh();

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

TEST(CGALMeshShellTests, test_path_on_surface) {

	ompl::RNG rng(42);

	auto mesh = mkConvexMesh();

	CGALMeshShell shell(mesh, 0, 0);

	for (size_t i = 0; i < 100; ++i) {

		Eigen::Vector3d start(rng.uniformReal(-1, 1), rng.uniformReal(-1, 1), rng.uniformReal(-1, 1));
		Eigen::Vector3d goal(rng.uniformReal(-1, 1), rng.uniformReal(-1, 1), rng.uniformReal(-1, 1));

		auto start1 = shell.nearest_point_on_shell(start);
		auto goal1 = shell.nearest_point_on_shell(goal);

		auto path = std::dynamic_pointer_cast<PiecewiseLinearPath<CGALMeshShellPoint>>(shell.path_from_to(start1, goal1));

		for (auto & point : path->points) {
			Eigen::Vector3d carthesian = shell.surface_point(point);

			Eigen::Vector3d reprojected = shell.surface_point(shell.nearest_point_on_shell(carthesian));

			ASSERT_LT((carthesian - reprojected).norm(), 1e-6);
		}

	}

}

bool bool_xor(bool a, bool b) {
	return (a || b) && !(a && b);
}

TEST(CGALMeshShellTests, test_path_rotate_xor_move) {

	ompl::RNG rng(42);

	auto mesh = mkConvexMesh();

	CGALMeshShell shell(mesh, 0, 0);

	for (size_t i = 0; i < 100; ++i) {

		Eigen::Vector3d start(rng.uniformReal(-1, 1), rng.uniformReal(-1, 1), rng.uniformReal(-1, 1));
		Eigen::Vector3d goal(rng.uniformReal(-1, 1), rng.uniformReal(-1, 1), rng.uniformReal(-1, 1));

		auto start1 = shell.nearest_point_on_shell(start);
		auto goal1 = shell.nearest_point_on_shell(goal);

		auto path = std::dynamic_pointer_cast<PiecewiseLinearPath<CGALMeshShellPoint>>(shell.path_from_to(start1, goal1));

		for (size_t i = 0; i < path->points.size() - 1; ++i) {
			auto & p1 = path->points[i];
			auto & p2 = path->points[i + 1];

			Eigen::Vector3d carthesian1 = shell.surface_point(p1);
			Eigen::Vector3d carthesian2 = shell.surface_point(p2);

			Eigen::Vector3d normal1 = shell.normalAt(p1);
			Eigen::Vector3d normal2 = shell.normalAt(p2);

			double angle = acos(std::clamp(normal1.dot(normal2), -1.0, 1.0));
			double distance = (carthesian1 - carthesian2).norm();

			std::cout << "i: " << i << " angle: " << angle << " distance: " << distance << std::endl;

			ASSERT_TRUE(angle < 1e-6 | distance < 1e-6);

		}

	}

}


TEST(CGALMeshShellTests, test_surface_point_stable) {

	/*
	 * In this test, we generate random points in the mesh.
	 *
	 * We then convert them to carthesian coordinates, and then back to the surface, then back to carthesian coordinates.
	 *
	 * The point should remain in place, within some margin.
	 */

	ompl::RNG rng(42);

	auto mesh = mkConvexMesh();

	CGALMeshShell shell(mesh, 0, 0.5);

	for (size_t i = 0; i < 100; ++i) {

		Eigen::Vector3d start(rng.uniformReal(-10, 10), rng.uniformReal(-10, 10), rng.uniformReal(-10, 10));

		Eigen::Vector3d carthesian = shell.surface_point(shell.nearest_point_on_shell(start));

		for (size_t ii = 0; ii < 10; ++ii) {

			auto shellpoint = shell.nearest_point_on_shell(carthesian);

			Eigen::Vector3d reprojected = shell.surface_point(shellpoint);

			if ((carthesian - reprojected).norm() > 1e-6) {
				std::cout << "i: " << i << " ii: " << ii << " carthesian: " << carthesian.transpose() << " reprojected: " << reprojected.transpose() << " distance: " << (carthesian - reprojected).norm() << std::endl;
				std::cout << "Surface point normal: " << shellpoint.normal.transpose() << std::endl;
			}

			ASSERT_LT((carthesian - reprojected).norm(), 1e-6);

		}

	}

}

TEST(CGALMeshShellTests, test_surface_point_stable_noisy) {

	ompl::RNG rng(42);

	auto mesh = mkConvexMesh();

	CGALMeshShell shell(mesh, 0, 0.5);

	for (size_t i = 0; i < 100; ++i) {

		Eigen::Vector3d start(rng.uniformReal(-10, 10), rng.uniformReal(-10, 10), rng.uniformReal(-10, 10));

		Eigen::Vector3d carthesian = shell.surface_point(shell.nearest_point_on_shell(start));

		for (size_t ii = 0; ii < 10; ++ii) {

			Eigen::Vector3d noise(rng.uniformReal(-0.01, 0.01), rng.uniformReal(-0.01, 0.01), rng.uniformReal(-0.01, 0.01));

			Eigen::Vector3d reprojected = shell.surface_point(shell.nearest_point_on_shell(carthesian + noise));

			ASSERT_LT((carthesian - reprojected).norm(), noise.norm() * 2.0);
		}

	}

}