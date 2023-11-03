// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 19-9-23.
//

#ifndef MGODPL_MOVEIT_MOTION_VELOCITY_H
#define MGODPL_MOVEIT_MOTION_VELOCITY_H

namespace moveit {
	namespace core {
		class RobotState;
		class RevoluteJointModel;
		class JointModel;
	}
}

#include "../math/Vec3.h"
#include "moveit_forward_declarations.h"

namespace mgodpl {

	/**
	 * A velocity in 3D space consisting of a linear and an angular component.
	 */
	struct Velocity {
		math::Vec3d linear;		/// Linear velocity; translation over one time step.
		math::Vec3d angular;  /// Angular velocity; rotation over one time step.
	};

	/**
	 * \brief Computes the velocity of a point on a rigid body, in the parent frame.
	 *
	 * @param velocity 		The velocity of the rigid body, in the parent frame
	 * @param point 		The point on the rigid body, in local coordinates
	 * @return 				The velocity of the point
	 */
	math::Vec3d velocity_at_point(const Velocity &velocity, const math::Vec3d &point);

	/**
	 * \brief Computes the maximum linear velocity of any point on a rigid body at a given distance from the origin,
	 * 		  under a given velocity of the rigid body.
	 *
	 * @param velocity 		The velocity of the rigid body
	 * @param distance 		The distance from the origin
	 * @return 				The maximum linear velocity of any point on the rigid body at the given distance from the origin
	 */
	double max_velocity_at_distance(const Velocity &velocity, const double &distance);

	/**
	 * Calculate the velocity of a revolute joint between two robot states.
	 *
	 * Note that for non-continuous joints, the angle change may be more than pi.
	 *
	 * Effectively, it's the signed version of RevoluteJointModel::distance().
	 *
	 * @param state1 The initial robot state.
	 * @param state2 The final robot state.
	 * @param revJoint The revolute joint model.
	 * @return The velocity as a `Velocity` object.
	 */
	Velocity getRevoluteVelocity(const moveit::core::RobotState &state1,
								 const moveit::core::RobotState &state2,
								 const moveit::core::RevoluteJointModel *revJoint);

	/**
	 * Calculate the velocity of a transform-based joint between two robot states.
	 *
	 * @param joint The transform-based joint model.
	 * @param state1 The initial robot state.
	 * @param state2 The final robot state.
	 * @return The velocity as a `Velocity` object.
	 */
	Velocity getTransformBasedVelocity(const moveit::core::JointModel *joint,
									   const moveit::core::RobotState &state1,
									   const moveit::core::RobotState &state2);


	/**
	 * Assuming a linear interpolation between state1 and state2 and a time step of 1
	 * compute the velocity of joint in the frame of the attachment point of the joint.
	 *
	 * Ideally, this would be a method of JointModel, but I don't want to modify the MoveIt source code at the moment.
	 *
	 * @throws	std::runtime_error	If the joint type is unknown or unimplemented.
	 *
	 * @param joint  	The joint.
	 * @param state1 	The start state
	 * @param state2 	The end state
	 * @return 		 	The velocity of joint
	 */
	Velocity jointVelocity(const moveit::core::JointModel *joint,
						   const moveit::core::RobotState &state1,
						   const moveit::core::RobotState &state2);

	/**
	 * Assuming a linear interpolation between state1 and state2, what is the maximum velocity of any point on any collision object?
	 *
	 * @param state1 The start state
	 * @param state2 The end state
	 * @return 		 The maximum velocity of any point on any collision object
	 */
	double motionMaximumVelocity(const moveit::core::RobotState &state1, const moveit::core::RobotState &state2);

}

#endif //MGODPL_MOVEIT_MOTION_VELOCITY_H
