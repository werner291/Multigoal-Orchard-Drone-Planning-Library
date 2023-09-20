// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <gtest/gtest.h>
#include <geometric_shapes/shape_operations.h>
#include "../src/utilities/experiment_utils.h"
#include "../src/MyCollisionEnv.h"
#include "../src/utilities/moveit.h"
#include "../src/utilities/moveit_motion_velocity.h"
#include "../src/utilities/mesh_utils.h"

TEST(MyCollisionEnv, test1) {

	auto robot = loadRobotModel();
	auto tree_model = loadTreeMeshes("appletree");

	auto world = std::make_shared<collision_detection::World>();

	const shapes::ShapeConstPtr tree_shape(shapes::constructShapeFromMsg(tree_model.trunk_mesh));

	world->addToObject("tree", tree_shape, Eigen::Isometry3d::Identity());

	moveit::core::RobotState state(robot);
	state.setVariablePosition(0, 0.0);
	state.setVariablePosition(1, 0.0);
	state.setVariablePosition(2, 1.0);
	setBaseOrientation(state, Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0));
	state.setVariablePosition(7, 0.0);
	state.setVariablePosition(8, 0.0);
	state.setVariablePosition(9, 0.0);
	state.update(true);

	moveit::core::RobotState state1(state);
	state1.setVariablePosition(1, -10.0);
	state1.update(true);

	moveit::core::RobotState state2(state);
	state2.setVariablePosition(1, 10.0);
	state2.update(true);

	auto collision_env = std::make_shared<MyCollisionEnv>(robot, world);

	{
		collision_detection::CollisionRequest req;
		req.verbose = true;
		collision_detection::CollisionResult res;
		collision_env->collision_detection::CollisionEnvFCL::checkRobotCollision(req, res, state);
		ASSERT_TRUE(res.collision);
	}

	{
		collision_detection::CollisionRequest req;
		collision_detection::CollisionResult res;
		collision_env->collision_detection::CollisionEnvFCL::checkRobotCollision(req, res, state1);
		ASSERT_FALSE(res.collision);
	}

	{
		collision_detection::CollisionRequest req;
		collision_detection::CollisionResult res;
		collision_env->collision_detection::CollisionEnvFCL::checkRobotCollision(req, res, state2);
		ASSERT_FALSE(res.collision);
	}

	{
		MyCollisionEnv::ContinuousCollisionRequest req;
		req.distance_threshold = 0.01;

		MyCollisionEnv::ContinuousCollisionResult res;

		collision_env->checkRobotCollision(req, res, state1, state2);

		ASSERT_TRUE(res.collision);
		ASSERT_NEAR(res.time_of_contact, 0.5, 0.2);
	}

}

/**
 * Generates a random robot state with a random base translation.
 *
 * @param robot The robot model.
 */
moveit::core::RobotState randomRobotState(const moveit::core::RobotModelConstPtr &robot) {
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(-5.0, 5.0);

	moveit::core::RobotState state(robot);
	state.setToRandomPositions();
	setBaseTranslation(state, Eigen::Vector3d(dis(gen), dis(gen), dis(gen)));
	state.update(true);

	return state;
}

/**
 * Generate a random point inside the aabb of a link.
 */
Eigen::Vector3d randomPointInLinkAABB(const moveit::core::LinkModel *link) {
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(-1.0, 1.0);

	const auto aabb_halfextents = link->getShapeExtentsAtOrigin() / 2.0;
	const auto aabb_center = link->getCenteredBoundingBoxOffset();

	return Eigen::Vector3d(aabb_center.x() + aabb_halfextents.x() * dis(gen),
						   aabb_center.y() + aabb_halfextents.y() * dis(gen),
						   aabb_center.z() + aabb_halfextents.z() * dis(gen));
}

TEST(MyCollisionEnv, max_velocity) {

	auto robot = loadRobotModel();

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(-1.0, 1.0);

	for (int i = 0; i < 10000; ++i) {

		moveit::core::RobotState state1 = randomRobotState(robot);
		moveit::core::RobotState state2 = randomRobotState(robot);

		double max_velocity = mgodpl::motionMaximumVelocity(state1, state2);

		for (int j = 0; j < 10; ++j) {
			// Pick a random link.
			std::uniform_int_distribution<> link_dis(0, robot->getLinkModelNames().size() - 1);
			int link_index = link_dis(gen);

			// Pick a random point in the aabb of the link.
			const auto &link = robot->getLinkModels()[link_index];

			Eigen::Vector3d point = randomPointInLinkAABB(link);

			// Find the point back in the two states.
			Eigen::Vector3d point1 = state1.getGlobalLinkTransform(link) * point;
			Eigen::Vector3d point2 = state2.getGlobalLinkTransform(link) * point;

			// Find the velocity of the point; it should be less than the maximum velocity.
			EXPECT_LE((point2 - point1).norm(), max_velocity);
		}
	}
}

TEST(MyCollisionEnv, engineered_collision) {

	auto robot = loadRobotModel();
	auto tree_model = loadTreeMeshes("appletree");

	auto world = std::make_shared<collision_detection::World>();

	const shapes::ShapeConstPtr tree_shape(shapes::constructShapeFromMsg(tree_model.trunk_mesh));

	world->addToObject("tree", tree_shape, Eigen::Isometry3d::Identity());

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(-1.0, 1.0);

	for (int i = 0; i < 100; ++i) {
		// Generate two states and a random interpolation of the two.
		moveit::core::RobotState state1 = randomRobotState(robot);
		moveit::core::RobotState state2 = randomRobotState(robot);

		// Random interpolation.
		double t = std::uniform_real_distribution<>(0.0, 1.0)(gen);
		moveit::core::RobotState interpolated_state(robot);
		state1.interpolate(state2, t, interpolated_state);
		interpolated_state.update(true);

		// Pick a random link.
		std::uniform_int_distribution<> link_dis(0, robot->getLinkModelNames().size() - 1);
		int link_index = link_dis(gen);
		const auto &link = robot->getLinkModels()[link_index];

		// Pick a random point in the aabb of the link.
		Eigen::Vector3d point = interpolated_state.getGlobalLinkTransform(link) * randomPointInLinkAABB(link);

		// Pick a random point on the tree.

		size_t triangle_index = std::uniform_int_distribution<>(0, tree_model.trunk_mesh.triangles.size() - 1)(gen);
		const auto &triangle = tree_model.trunk_mesh.triangles[triangle_index];
		Eigen::Vector3d a = toEigen(tree_model.trunk_mesh.vertices[triangle.vertex_indices[0]]);
		Eigen::Vector3d b = toEigen(tree_model.trunk_mesh.vertices[triangle.vertex_indices[1]]);
		Eigen::Vector3d c = toEigen(tree_model.trunk_mesh.vertices[triangle.vertex_indices[2]]);

		Eigen::Vector3d tree_point = a + dis(gen) * (b - a) + dis(gen) * (c - a);

		// Apply a common translation to all three states to make the point on the tree collide with the point on the robot.
		Eigen::Vector3d translation = tree_point - point;

		setBaseTranslation(state1, getBaseTranslation(state1) + translation);
		setBaseTranslation(state2, getBaseTranslation(state2) + translation);
		setBaseTranslation(interpolated_state, getBaseTranslation(interpolated_state) + translation);

		state1.update(true);
		state2.update(true);
		interpolated_state.update(true);

		// Perform CCD.
		auto collision_env = std::make_shared<MyCollisionEnv>(robot, world);

		MyCollisionEnv::ContinuousCollisionRequest req;
		req.distance_threshold = 0.01;

		MyCollisionEnv::ContinuousCollisionResult res;

		collision_env->checkRobotCollision(req, res, state1, state2);

		ASSERT_TRUE(res.collision);

		// Check that the time of contact is close to the interpolation time, or before.
		// We leave a generous margin of 0.1.
		ASSERT_LE(res.time_of_contact, t + 0.1);

		std::cout << "Time of contact: " << res.time_of_contact << " (t = " << t << ")" << std::endl;
	}

}