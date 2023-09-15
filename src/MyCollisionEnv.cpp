// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 30-8-23.
//

#include "MyCollisionEnv.h"

struct Velocity {
	Eigen::Vector3d linear;
	Eigen::AngleAxisd angular;
};

/**
 * Computes the velocity of a point on a rigid body, in the parent frame.
 * @param velocity 		The velocity of the rigid body
 * @param point 		The point on the rigid body
 * @return 				The velocity of the point
 */
Eigen::Vector3d velocity_at_point(
		const Velocity &velocity,
		const Eigen::Vector3d &point
		) {

	// TODO: double-check.
	return velocity.linear + velocity.angular.axis().cross(point) * velocity.angular.angle();

}

/**
 * Assuming a linear interpolation between state1 and state2, compute the velocity of joint.
 *
 * @param joint  The joint
 * @param state1 The start state
 * @param state2 The end state
 * @return 		 The velocity of joint
 */
Velocity jointVelocity(
		const moveit::core::JointModel* joint,
		const moveit::core::RobotState &state1,
		const moveit::core::RobotState &state2
		) {

	// Make sure it's the same robot model.
	assert(state1.getRobotModel() == state2.getRobotModel());

	// We assume a constant-velocity interpolation between state1 and state2, so we don't need velocities.
	assert(!state1.hasVelocities() && !state2.hasVelocities());

	switch (joint->getType()) {

		case moveit::core::JointModel::UNKNOWN:
			throw std::runtime_error("Unknown joint type");
			break;

		case moveit::core::JointModel::REVOLUTE: {
			// Must treat separately because revolute joints may rotate more than 2pi.

			Eigen::Vector3d axis = ((const moveit::core::RevoluteJointModel *) joint)->getAxis();
			double angle = state2.getVariablePosition(joint->getFirstVariableIndex()) -
						   state1.getVariablePosition(joint->getFirstVariableIndex());

			return {.linear = Eigen::Vector3d::Zero(), .angular = Eigen::AngleAxisd(angle, axis)};
		}
			break;

		case moveit::core::JointModel::FLOATING: {

			Eigen::Isometry3d tf1;
			((const moveit::core::FloatingJointModel *) joint)->computeTransform(
					state1.getVariablePositions() + joint->getFirstVariableIndex(), tf1);
			Eigen::Isometry3d tf2;
			((const moveit::core::FloatingJointModel *) joint)->computeTransform(
					state2.getVariablePositions() + joint->getFirstVariableIndex(), tf2);

			Eigen::Isometry3d tf = tf1.inverse() * tf2;

			return {
				.linear = tf.translation(),
				.angular = Eigen::AngleAxisd(tf.rotation())
			};
		}

			break;

		default:
			throw std::runtime_error("Unimplemented joint type");
			break;
	}

}

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
		const double parentMaxVelocity = 0.0;
		const moveit::core::JointModel* frame;
	};

	std::vector<Frame> stack {
		{0.0, robot_model->getRootJoint()}
	};

	double maxVelocity = 0.0;

	while (!stack.empty()) {

		Frame frame = stack.back();
		stack.pop_back();

		Velocity velocity = jointVelocity(frame.frame, state1, state2);

	}
}

MyCollisionEnv::MyCollisionEnv(const moveit::core::RobotModelConstPtr &model,
							   const collision_detection::WorldPtr &world,
							   double padding,
							   double scale) : CollisionEnvFCL(model, world, padding, scale) {

}

void MyCollisionEnv::checkRobotCollision(const collision_detection::CollisionRequest &req,
										 collision_detection::CollisionResult &res,
										 const moveit::core::RobotState &state1,
										 const moveit::core::RobotState &state2,
										 const collision_detection::AllowedCollisionMatrix &acm) const {
	CollisionEnvFCL::checkRobotCollision(req, res, state1, state2, acm);
}

void MyCollisionEnv::checkRobotCollision(const collision_detection::CollisionRequest &req,
										 collision_detection::CollisionResult &res,
										 const moveit::core::RobotState &state1,
										 const moveit::core::RobotState &state2) const {
	CollisionEnvFCL::checkRobotCollision(req, res, state1, state2);
}
