// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 30-8-23.
//

#include <geometric_shapes/shapes.h>
#include "MyCollisionEnv.h"

/**
 * A velocity in 3D space consisting of a linear and an angular component.
 */
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
Eigen::Vector3d velocity_at_point(const Velocity &velocity, const Eigen::Vector3d &point) {
	// TODO: double-check.
	return velocity.linear + velocity.angular.axis().cross(point) * velocity.angular.angle();

}

double max_velocity_at_distance(const Velocity &velocity, const double &distance) {

	assert(std::abs(velocity.angular.axis().squaredNorm() - 1.0) < 1e-6);

	return velocity.linear.norm() + velocity.angular.angle() * distance;

}

/**
 * Assuming a linear interpolation between state1 and state2, compute the velocity of joint.
 *
 * @param joint  The joint
 * @param state1 The start state
 * @param state2 The end state
 * @return 		 The velocity of joint
 */
Velocity jointVelocity(const moveit::core::JointModel *joint,
					   const moveit::core::RobotState &state1,
					   const moveit::core::RobotState &state2) {

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

			return {.linear = tf.translation(), .angular = Eigen::AngleAxisd(tf.rotation())};
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
double motionMaximumVelocity(const moveit::core::RobotState &state1, const moveit::core::RobotState &state2) {

	assert(state1.getRobotModel() == state2.getRobotModel());

	const auto &robot_model = state1.getRobotModel();

	struct Frame {
		const double parentMaxLinearVelocity;
		const double parentMaxAngularVelocity;
		const moveit::core::JointModel *frame;
	};

	std::vector<Frame> stack{{
		0.0, 0.0, robot_model->getRootJoint()
	}};

	double maxVelocity = 0.0;

	while (!stack.empty()) {

		Frame frame = stack.back();
		stack.pop_back();

		Velocity velocity = jointVelocity(frame.frame, state1, state2);

		// First, look at the joint's own collision geometry.
		const moveit::core::LinkModel *link = frame.frame->getChildLinkModel();

		// I don't know why LinkModel stores these separately instead of just storing an AlignedBox3d.
		// TODO: double-check that these are the right ones.
		const auto aabb_halfextents = link->getShapeExtentsAtOrigin() / 2.0;
		const auto aabb_center = link->getCenteredBoundingBoxOffset();

		Eigen::Vector3d furthest_point_abs(
				std::max(abs(aabb_center.x() + aabb_halfextents.x()), abs(aabb_center.x() - aabb_halfextents.x())),
				std::max(abs(aabb_center.y() + aabb_halfextents.y()), abs(aabb_center.y() - aabb_halfextents.y())),
				std::max(abs(aabb_center.z() + aabb_halfextents.z()), abs(aabb_center.z() - aabb_halfextents.z()))
				);

		double this_link_local_max_velocity = max_velocity_at_distance(velocity, furthest_point_abs.norm());
		double global_max_velocity = this_link_local_max_velocity + frame.parentMaxLinearVelocity +
									 frame.parentMaxAngularVelocity * furthest_point_abs.norm();

		if (global_max_velocity > maxVelocity) {
			maxVelocity = global_max_velocity;
		}

		// Then push the children onto the stack.
		for (const auto &child : link->getChildJointModels()) {
			// Double-check if this is right...
			stack.push_back({
				frame.parentMaxAngularVelocity + link->getJointOriginTransform().translation().norm() * frame.parentMaxLinearVelocity + velocity.linear.norm(),
				frame.parentMaxAngularVelocity + velocity.angular.angle(),
				child
			});
		}

	}

	return maxVelocity;
}

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
	double maxVelocity = motionMaximumVelocity(state1, state2);

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
