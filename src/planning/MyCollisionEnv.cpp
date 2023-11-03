// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <geometric_shapes/shapes.h>
#include "MyCollisionEnv.h"
#include "moveit_motion_velocity.h"
#include "moveit_forward_declarations.h"

MyCollisionEnv::MyCollisionEnv(const moveit::core::RobotModelConstPtr &model,
							   const collision_detection::WorldPtr &world,
							   double padding,
							   double scale) : CollisionEnvFCL(model, world, padding, scale) {

}

void MyCollisionEnv::checkRobotCollision(const MyCollisionEnv::ContinuousCollisionRequest &req,
										 MyCollisionEnv::ContinuousCollisionResult &res,
										 const moveit::core::RobotState &state1,
										 const moveit::core::RobotState &state2) const {

	// First, check if the robot is in collision at the start state.
	collision_detection::CollisionRequest collisionRequest;
	collision_detection::CollisionResult collisionResult;

	this->collision_detection::CollisionEnvFCL::checkRobotCollision(collisionRequest, collisionResult, state1);

	if (collisionResult.collision) {
		res.collision = true;
		res.time_of_contact = 0.0;
		return;
	}

	// Use conservative advancement.
	double maxVelocity = mgodpl::motionMaximumVelocity(state1, state2);

	double time = 0.0;

	moveit::core::RobotState interpolatedState(state1.getRobotModel());

	while (time < 1.0) {

		state1.interpolate(state2, time, interpolatedState);
		interpolatedState.update(true);

		collision_detection::DistanceRequest distanceRequest;

		distanceRequest.verbose = true;
		distanceRequest.enable_nearest_points = true;

		distanceRequest.distance_threshold = (1.0 - time) / maxVelocity;

		collision_detection::DistanceResult distanceResult;

		this->distanceRobot(distanceRequest, distanceResult, interpolatedState);

		if (distanceResult.minimum_distance.distance < req.distance_threshold) {
			res.collision = true;
			res.time_of_contact = time;
			return;
		} else {
			time += distanceResult.minimum_distance.distance / maxVelocity;
		}
	}

	res.collision = false;

}
