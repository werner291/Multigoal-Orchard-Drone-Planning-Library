// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 30-8-23.
//

#include "MyCollisionEnv.h"

/**
 * Assuming a linear interpolation between state1 and state2, what is the maximum velocity of any point on any collision object?
 *
 * @param state1 The start state
 * @param state2 The end state
 * @return 		 The maximum velocity of any point on any collision object
 */
double motionMaximumVelocity(
		const moveit::core::RobotState &state1,
		const moveit::core::RobotState &state2
		) {

	assert(state1.getRobotModel() == state2.getRobotModel());

	const auto &robot_model = state1.getRobotModel();

	struct Frame {
		const double maxVelocity = 0.0;
		const moveit::core::JointModel* frame;
	};

	std::vector<Frame> stack;

	robot_model->getRootJoint()

}

void MyCollisionEnv::checkRobotCollision(const collision_detection::CollisionRequest &req,
										 collision_detection::CollisionResult &res,
										 const moveit::core::RobotState &state1,
										 const moveit::core::RobotState &state2,
										 const collision_detection::AllowedCollisionMatrix &acm) const {



}

void MyCollisionEnv::checkRobotCollision(const collision_detection::CollisionRequest &req,
										 collision_detection::CollisionResult &res,
										 const moveit::core::RobotState &state1,
										 const moveit::core::RobotState &state2) const {
	CollisionEnvFCL::checkRobotCollision(req, res, state1, state2);
}
