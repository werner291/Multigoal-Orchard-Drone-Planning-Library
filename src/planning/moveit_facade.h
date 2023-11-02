// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef MGODPL_MOVEIT_FACADE_H
#define MGODPL_MOVEIT_FACADE_H

#include "../math/Vec3.h"

#include <random>
#include <vector>
#include <memory>

// Forward declarations of MoveIt stuff. We don't import them because MoveIt likes
// to import a lot of stuff (Mostly Eigen) that we don't want to be imported everywhere.
namespace moveit::core {
	class RobotModel;
	using RobotModelConstPtr = std::shared_ptr<const RobotModel>;
	class RobotState;
}

/**
 * This namespace contains a few functions that make it easier to interface with MoveIt,
 * and specifically avoid dealing with Eigen.
 */
namespace mgodpl::moveit_facade {

	/**
	 * A point in joint space. This is a simple wrapper around a vector of doubles.
	 *
	 * It provides a lightweight alternative to RobotState, which imports a lot of stuff
	 * from Eigen that we don't want to be imported everywhere.
	 *
	 * Note: You need a RobotModel to interpret these values.
	 */
	struct JointSpacePoint {

		/// The joint values.
		std::vector<double> joint_values;

		/// Construct a point from a vector of joint values.
		JointSpacePoint(std::vector<double> joint_values) : joint_values(std::move(joint_values)) {}

		/// Write the joint values to the position variables of the given RobotState.
		/// @param compute_fk 		Whether to compute forward kinematics (call RobotState::update) after setting the joint values.
		void to_moveit(moveit::core::RobotState &state, bool compute_fk = true) const;

		/// Construct a point from the position variables of the given RobotState.
		static JointSpacePoint from_moveit(const moveit::core::RobotState& state);
	};

	/**
	 * Compute the position of the end-effector of the robot in the given state, assumed to be named "end_effector".
	 *
	 * Note: This function will compute the full forward kinematics; you should cache
	 * the result if you need it multiple times, or for multiple links.
	 *
	 * @param robot 		The robot model.
	 * @param state 		The joint-space state.
	 * @return 				The position of the end-effector.
	 */
	math::Vec3d computeEndEffectorPosition(const moveit::core::RobotModel& robot, const JointSpacePoint &state);

	/**
	 * The distance between two points in joint space, as defined by RobotModel::distance.
	 *
	 * @param robot 		The robot model.
	 * @param a 			One point.
	 * @param b 			The other point.
	 * @return 				The distance between the two points.
	 */
	double moveit_joint_distance(const moveit::core::RobotModel &robot,
								 const JointSpacePoint &a,
								 const JointSpacePoint &b);

	/**
	 * Interpolate between two points in joint space, consistent with RobotModel::interpolate.
	 *
	 * @param robot 		The robot model.
	 * @param a 			The first point.
	 * @param b 			The second point.
	 * @param t 			The interpolation parameter.
	 * @return 				The interpolated point.
	 */
	JointSpacePoint interpolate(const moveit::core::RobotModel &robot,
								const JointSpacePoint &a,
								const JointSpacePoint &b,
								double t);
}

#endif //MGODPL_MOVEIT_FACADE_H
