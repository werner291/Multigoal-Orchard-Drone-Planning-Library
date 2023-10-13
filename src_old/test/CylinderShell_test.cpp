#include <gtest/gtest.h>
#include <cmath>
#include <Eigen/Core>
#include "../src/shell_space/CylinderShell.h"

class CylinderShellTest : public ::testing::Test {
protected:
	CylinderShellTest()
			: shell(5.0, Eigen::Vector2d(0, 0)) {
	}

	CylinderShell shell;
};

TEST_F(CylinderShellTest, ArmVectorInward) {
	CylinderShellPoint p1{0, 0};
	CylinderShellPoint p2{M_PI / 2, 0};
	CylinderShellPoint p3{M_PI, 0};
	CylinderShellPoint p4{3 * M_PI / 2, 0};

	Eigen::Vector3d arm_vector1 = shell.arm_vector(p1);
	Eigen::Vector3d arm_vector2 = shell.arm_vector(p2);
	Eigen::Vector3d arm_vector3 = shell.arm_vector(p3);
	Eigen::Vector3d arm_vector4 = shell.arm_vector(p4);

	Eigen::Vector3d surface_point1 = (shell.surface_point(p1) - Eigen::Vector3d(shell.center.x(), shell.center.y(), 0.0)) / shell.radius;
	Eigen::Vector3d surface_point2 = (shell.surface_point(p2) - Eigen::Vector3d(shell.center.x(), shell.center.y(), 0.0)) / shell.radius;
	Eigen::Vector3d surface_point3 = (shell.surface_point(p3) - Eigen::Vector3d(shell.center.x(), shell.center.y(), 0.0)) / shell.radius;
	Eigen::Vector3d surface_point4 = (shell.surface_point(p4) - Eigen::Vector3d(shell.center.x(), shell.center.y(), 0.0)) / shell.radius;

	// Check if the arm vectors point inwards (towards the cylinder's axis)
	ASSERT_NEAR(arm_vector1.dot(surface_point1), -1, 0.01);
	ASSERT_NEAR(arm_vector2.dot(surface_point2), -1, 0.01);
	ASSERT_NEAR(arm_vector3.dot(surface_point3), -1, 0.01);
	ASSERT_NEAR(arm_vector4.dot(surface_point4), -1, 0.01);

	for (int i = 0; i < 100; ++i) {

		double theta = 2 * M_PI * i / 100;

		CylinderShellPoint p{theta, 0};
		Eigen::Vector3d arm_vector = shell.arm_vector(p);
		Eigen::Vector3d surface_point = (shell.surface_point(p) - Eigen::Vector3d(shell.center.x(), shell.center.y(), 0.0)) / shell.radius;

		ASSERT_NEAR(arm_vector.dot(surface_point), -1, 0.01);

	}
}


TEST_F(CylinderShellTest, NearestPointOnShell) {
	// Test for a specific point in R^3
	Eigen::Vector3d p(0, 2 * shell.radius, 2);
	CylinderShellPoint nearest = shell.nearest_point_on_shell(p);
	ASSERT_NEAR(M_PI / 2, nearest.angle, 1e-6);
	ASSERT_NEAR(2, nearest.height, 1e-6);

	// Test with random points in R^3
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(-10, 10);
	for (int i = 0; i < 10; ++i) {
		Eigen::Vector3d random_p(dis(gen), dis(gen), dis(gen));
		CylinderShellPoint random_nearest = shell.nearest_point_on_shell(random_p);
		ASSERT_NEAR(atan2(random_p.y(), random_p.x()), random_nearest.angle, 1e-6);
		ASSERT_NEAR(random_p.z(), random_nearest.height, 1e-6);
	}
}

TEST_F(CylinderShellTest, SurfacePoint) {
	// Test for a specific angle (M_PI / 2)
	double angle = M_PI / 2;
	CylinderShellPoint p{angle, 2};
	Eigen::Vector3d surface_pt = shell.surface_point(p);
	ASSERT_NEAR(0, surface_pt.x(), 1e-6);
	ASSERT_NEAR(shell.radius, surface_pt.y(), 1e-6);
	ASSERT_NEAR(2, surface_pt.z(), 1e-6);

	// Test with random angles
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> angle_dis(0, 2 * M_PI);
	std::uniform_real_distribution<> height_dis(-10, 10);
	for (int i = 0; i < 10; ++i) {
		double random_angle = angle_dis(gen);
		double random_height = height_dis(gen);
		CylinderShellPoint random_p{random_angle, random_height};
		Eigen::Vector3d random_surface_pt = shell.surface_point(random_p);
		ASSERT_NEAR(cos(random_angle) * shell.radius, random_surface_pt.x(), 1e-6);
		ASSERT_NEAR(sin(random_angle) * shell.radius, random_surface_pt.y(), 1e-6);
		ASSERT_NEAR(random_height, random_surface_pt.z(), 1e-6);
	}
}

TEST_F(CylinderShellTest, PathFromTo) {
	// Test for specific points on the shell
	CylinderShellPoint from{M_PI / 2, 0};
	CylinderShellPoint to{3 * M_PI / 2, 4};
	auto path = shell.path_from_to(from, to);
	ASSERT_TRUE(path != nullptr);

	// Test with random points on the shell
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> angle_dis(0, 2 * M_PI);
	std::uniform_real_distribution<> height_dis(-10, 10);
	for (int i = 0; i < 10; ++i) {
		double random_angle_from = angle_dis(gen);
		double random_height_from = height_dis(gen);
		double random_angle_to = angle_dis(gen);
		double random_height_to = height_dis(gen);
		CylinderShellPoint random_from{random_angle_from, random_height_from};
		CylinderShellPoint random_to{random_angle_to, random_height_to};
		auto random_path = shell.path_from_to(random_from, random_to);
		ASSERT_TRUE(random_path != nullptr);
	}
}

TEST_F(CylinderShellTest, PathLength) {
	CylinderShellPoint from{M_PI / 2, 0};
	CylinderShellPoint to{M_PI, 3};
	auto path = shell.path_from_to(from, to);
	double length = shell.path_length(path);

	double angle_arc = (M_PI / 2) * shell.radius; // 2 * M_PI / 4 = M_PI / 2
	double height_delta = 3;
	double expected_length = std::sqrt(angle_arc * angle_arc + height_delta * height_delta);

	ASSERT_NEAR(expected_length, length, 1e-4);
}

TEST_F(CylinderShellTest, RandomNear) {
	CylinderShellPoint p{M_PI / 2, 2};
	double radius = 1.0;
	CylinderShellPoint near_pt = shell.random_near(p, radius);
	ASSERT_LE(std::abs(near_pt.angle - p.angle), radius);
	ASSERT_LE(std::abs(near_pt.height - p.height), radius);
}
