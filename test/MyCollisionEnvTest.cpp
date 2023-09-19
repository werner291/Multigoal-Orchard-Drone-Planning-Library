// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <gtest/gtest.h>
#include <geometric_shapes/shape_operations.h>
#include "../src/utilities/experiment_utils.h"
#include "../src/MyCollisionEnv.h"
#include "../src/utilities/moveit.h"

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