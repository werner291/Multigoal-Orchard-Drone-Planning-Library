// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <gtest/gtest.h>
#include <geometric_shapes/shape_operations.h>
#include "../src/utilities/experiment_utils.h"
#include "../src/MyCollisionEnv.h"
#include "../src/utilities/moveit.h"
#include "../src/utilities/moveit_motion_velocity.h"

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

TEST(MyCollisionEnv, max_velocity) {

	auto robot = loadRobotModel();

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(-1.0, 1.0);

	for (int i = 0; i < 100000; ++i) {

		moveit::core::RobotState state1(robot);
		state1.setToRandomPositions();
		setBaseTranslation(state1, Eigen::Vector3d(dis(gen), dis(gen), dis(gen)));
		state1.update(true);


		moveit::core::RobotState state2(robot);
		state2.setToRandomPositions();
		setBaseTranslation(state2, Eigen::Vector3d(dis(gen), dis(gen), dis(gen)));
		state2.update(true);

		double max_velocity = mgodpl::motionMaximumVelocity(state1, state2);

		// Pick a random link.
		std::uniform_int_distribution<> link_dis(0, robot->getLinkModelNames().size() - 1);
		int link_index = 2;//TODO link_dis(gen);

		// Pick a random point in the aabb of the link.
		const auto &link = robot->getLinkModels()[link_index];
		const auto aabb_halfextents = link->getShapeExtentsAtOrigin() / 2.0;
		const auto aabb_center = link->getCenteredBoundingBoxOffset();

		Eigen::Vector3d point(aabb_center.x() + aabb_halfextents.x() * dis(gen),
							  aabb_center.y() + aabb_halfextents.y() * dis(gen),
							  aabb_center.z() + aabb_halfextents.z() * dis(gen));

		// Find the point back in the two states.
		Eigen::Vector3d point1 = state1.getGlobalLinkTransform(link) * point;
		Eigen::Vector3d point2 = state2.getGlobalLinkTransform(link) * point;

		// Find the velocity of the point.
		EXPECT_LE((point2 - point1).norm(), max_velocity);

		if ((point2 - point1).norm() > max_velocity) {
			std::cout << "max_velocity: " << max_velocity << std::endl;
			std::cout << "Link name: " << link->getName() << std::endl;
		}
	}

}