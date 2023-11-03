// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 19-9-23.
//

#include "moveit_motion_velocity.h"
#include "moveit_forward_declarations.h"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace mgodpl {
	using namespace math;

	Vec3d velocity_at_point(const Velocity &velocity, const Vec3d &point) {
		return velocity.linear + velocity.angular.cross(point);
	}

	double max_velocity_at_distance(const Velocity &velocity, const double &distance) {

		return velocity.linear.norm() + velocity.angular.norm() * distance;

	}

	Velocity getRevoluteVelocity(const moveit::core::RobotState &state1,
								 const moveit::core::RobotState &state2,
								 const moveit::core::RevoluteJointModel *revJoint) {

		const Vec3d axis(revJoint->getAxis().data());

		// Calculate the angle change between the two states.
		double angle = state2.getVariablePosition(revJoint->getFirstVariableIndex()) -
					   state1.getVariablePosition(revJoint->getFirstVariableIndex());

		if (revJoint->isContinuous()) {
			// Ensure the angle is within the range [-pi, pi] for continuous joints.
			angle = fmod(angle + M_PI, 2 * M_PI) - M_PI;
		}

		return {
				// The linear velocity is zero for revolute joints.
				.linear = Vec3d::Zero(),
				// Assemble the angle and the axis into an AngleAxis object.
				.angular = axis * angle
		};
	}

	Velocity getTransformBasedVelocity(const moveit::core::JointModel *joint,
													   const moveit::core::RobotState &state1,
													   const moveit::core::RobotState &state2) {

		// Compute the transform of the joint in both states.
		Eigen::Isometry3d tf1, tf2;
		joint->computeTransform(state1.getVariablePositions() + joint->getFirstVariableIndex(), tf1);
		joint->computeTransform(state2.getVariablePositions() + joint->getFirstVariableIndex(), tf2);

		// Compute the relative transform between the two states.
		Eigen::Isometry3d tf = tf1.inverse() * tf2;

		Eigen::AngleAxisd rot(tf.rotation());

		// The velocity is the relative transform between the two states, assumed to occur over one time unit.
		// We break it down into a linear and an angular component.
		return {
			.linear = Vec3d(tf.translation().data()),
			.angular = Vec3d(rot.axis().data()) * rot.angle()
		};
	}

	Velocity jointVelocity(const moveit::core::JointModel *joint,
										   const moveit::core::RobotState &state1,
										   const moveit::core::RobotState &state2) {

		// Make sure it's the same robot model.
		assert(state1.getRobotModel() == state2.getRobotModel());

		// We assume a constant-velocity interpolation between state1 and state2, so we don't need velocities.
		assert(!state1.hasVelocities() && !state2.hasVelocities());

		// Different joints work differently.
		// Especially revolute joints have to be treated specially since the rotation may be more than pi.
		// If it is, we must NOT treat it as a rotation of less than pi in the other direction.
		switch (joint->getType()) {

			case moveit::core::JointModel::UNKNOWN:
				throw std::runtime_error("Unknown joint type");
				break;

			case moveit::core::JointModel::REVOLUTE:
				return getRevoluteVelocity(state1, state2, (const moveit::core::RevoluteJointModel *) joint);

			case moveit::core::JointModel::PRISMATIC:
			case moveit::core::JointModel::FLOATING:
			case moveit::core::JointModel::FIXED:
				return getTransformBasedVelocity(joint, state1, state2);
				break;

			default:
				// Don't really know how to treat planar joints.
				throw std::runtime_error("Unimplemented joint type");
				break;
		}

	}

	/**
	 * Calculate the coordinates of the furthest point from the origin within a bounding box associated with a LinkModel.
	 *
	 * @param link A pointer to the LinkModel for which the furthest point is to be calculated.
	 *
	 * @return An Vec3d representing the coordinates of the furthest point within the bounding box.
	 */
	Vec3d furthestPointFromOriginInLink(const moveit::core::LinkModel *link) {

		// Calculate half extents and centered offset of the Axis-Aligned Bounding Box (AABB)
		const auto aabbHalfextents = link->getShapeExtentsAtOrigin() / 2.0;
		const Vec3d aabbCenter(link->getCenteredBoundingBoxOffset().data());

		// Get one of the corners of the AABB that is furthest away from the origin.
		Vec3d furthestPointAbs(aabbCenter.x() > 0 ? aabbHalfextents.x() : -aabbHalfextents.x(),
							   aabbCenter.y() > 0 ? aabbHalfextents.y() : -aabbHalfextents.y(),
							   aabbCenter.z() > 0 ? aabbHalfextents.z() : -aabbHalfextents.z());

		// Add the centered offset to obtain the absolute coordinates of the furthest point
		return furthestPointAbs + aabbCenter;
	}


	double motionMaximumVelocity(const moveit::core::RobotState &state1,
								 const moveit::core::RobotState &state2) {

		assert(state1.getRobotModel() == state2.getRobotModel());

		const auto &robot_model = state1.getRobotModel();

		struct Frame {
			// The maximum linear velocity of the parent link.
			const double parentMaxLinearVelocity;
			// The maximum angular velocity of the parent link.
			const double parentMaxAngularVelocity;
			// The joint model.
			const moveit::core::JointModel *frame;
		};

		std::vector<Frame> stack{{0.0, 0.0, robot_model->getRootJoint()}};

		double maxVelocity = 0.0;

		while (!stack.empty()) {

			Frame frame = stack.back();
			stack.pop_back();

			// Get the velocity of the link relative to the attachment frame of the joint.
			Velocity linkVelocity = jointVelocity(frame.frame, state1, state2);

			const moveit::core::LinkModel *link = frame.frame->getChildLinkModel();

			double thisLinkMaxAngularVelocity = linkVelocity.angular.norm() + frame.parentMaxAngularVelocity;
			double thisLinkMaxLinearVelocity = linkVelocity.linear.norm() + frame.parentMaxLinearVelocity +
											   thisLinkMaxAngularVelocity *
											   link->getJointOriginTransform().translation().norm();


			// Find the furthest point from the origin within the AABB of the link,
			// in the frame of the link itself (without considering any relation to the parent link).
			Vec3d localFurthestPoint = furthestPointFromOriginInLink(link);

			double furthest_point_velocity =
					thisLinkMaxLinearVelocity + thisLinkMaxAngularVelocity * localFurthestPoint.norm();

			if (furthest_point_velocity > maxVelocity) {
				maxVelocity = furthest_point_velocity;
			}

			// Then push the children onto the stack.
			for (const auto &child: link->getChildJointModels()) {

				// Double-check if this is right...
				stack.push_back({.parentMaxLinearVelocity= thisLinkMaxLinearVelocity, .parentMaxAngularVelocity= thisLinkMaxAngularVelocity, .frame= child});
			}

		}

		return maxVelocity;
	}
}