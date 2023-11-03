// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/2/23.
//

#include "CollisionDetection.h"
#include "moveit_motion_velocity.h"
#include "MyCollisionEnv.h"
#include "moveit_forward_declarations.h"

#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include <geometric_shapes/shape_operations.h>

bool mgodpl::moveit_facade::CollisionDetection::collides(const mgodpl::moveit_facade::JointSpacePoint &state) {

	collision_detection::CollisionRequest request;
	collision_detection::CollisionResult result;

	moveit::core::RobotState robot_state(collision_env->getRobotModel());
	state.to_moveit(robot_state);

	collision_env->collision_detection::CollisionEnvFCL::checkRobotCollision(request, result, robot_state);

	return result.collision;

}

bool mgodpl::moveit_facade::CollisionDetection::collides_ccd(const mgodpl::moveit_facade::JointSpacePoint &state1,
															 const mgodpl::moveit_facade::JointSpacePoint &state2) {

	MyCollisionEnv::ContinuousCollisionRequest request;
	MyCollisionEnv::ContinuousCollisionResult result;

	request.distance_threshold = 0.01;

	moveit::core::RobotState robot_state1(collision_env->getRobotModel());
	moveit::core::RobotState robot_state2(collision_env->getRobotModel());

	state1.to_moveit(robot_state1);
	state2.to_moveit(robot_state2);

	collision_env->checkRobotCollision(request, result, robot_state1, robot_state2);

	return result.collision;

}

mgodpl::moveit_facade::CollisionDetection::CollisionDetection(const std::vector<shape_msgs::msg::Mesh> &obstacle_meshes,
															  const moveit::core::RobotModelConstPtr robot) {

	this->collision_env = std::make_shared<MyCollisionEnv>(robot, std::make_shared<collision_detection::World>(), 0.0, 1.0);

	// Add the obstacles to the collision environment.
	for (const auto &mesh : obstacle_meshes) {
		this->collision_env->getWorld()->addToObject(
				"static_obstacles",
				shapes::ShapeConstPtr(shapes::constructShapeFromMsg(mesh)),
				Eigen::Isometry3d::Identity()
				);
	}

}
