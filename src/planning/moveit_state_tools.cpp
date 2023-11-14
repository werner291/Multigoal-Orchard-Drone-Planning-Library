// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/1/23.
//

#include "moveit_state_tools.h"
#include "../math/Vec3.h"
#include "JointSpacePoint.h"
#include "moveit_forward_declarations.h"
#include "../math/Polar.h"

#include <moveit/robot_state/robot_state.h>

#include <random>
#include <Eigen/Core>

namespace mgodpl::experiment_state_tools {

	using namespace math;

	moveit_facade::JointSpacePoint randomStateOutsideTree(const moveit::core::RobotModel &robot, const int seed) {

		moveit_facade::JointSpacePoint state = randomUprightWithBase(robot, 0.0, seed);

		// Get the Rng
		random_numbers::RandomNumberGenerator rng(seed);

		double random_t = rng.uniformReal(-M_PI, M_PI);
		double height = rng.uniformReal(0.5, 5.0);

		double radius = 4.0;

		state.joint_values[0] += radius * cos(random_t);
		state.joint_values[1] += radius * sin(random_t);
		state.joint_values[2] += height;

		return state;
	}

	void moveEndEffectorToPoint(const moveit::core::RobotModel &robot,
								moveit_facade::JointSpacePoint &state,
								const math::Vec3d &target) {

		// Get the end-effector position.
		math::Vec3d ee_pos = moveit_facade::computeEndEffectorPosition(robot, state);

		// Get the direction to the target.
		math::Vec3d delta = target - ee_pos;

		// Get the distance to the target.

		state.joint_values[0] += delta.x();
		state.joint_values[1] += delta.y();
		state.joint_values[2] += delta.z();

	}

	moveit_facade::JointSpacePoint
	randomUprightWithBase(const moveit::core::RobotModel &robot, double translation_bound, const int seed) {

		moveit::core::RobotModelConstPtr fake_shared_ptr(&robot, [](const moveit::core::RobotModel *) {});
		moveit::core::RobotState state(fake_shared_ptr);

		// Set the state to uniformly random values.
		// Unfortunately, this puts the base at the origin and the rotation will not be upright. We need to fix that.
		state.setToRandomPositions();

		// Get an Rng
		random_numbers::RandomNumberGenerator rng(seed);

		// Randomize the floating base within a box defined by the translation_bound.
		double *pos = state.getVariablePositions();
		pos[0] = rng.uniformReal(-translation_bound, translation_bound);
		pos[1] = rng.uniformReal(-translation_bound, translation_bound);
		pos[2] = rng.uniformReal(0 /* Do not put it underground */, translation_bound);

		// Compute a random yaw-rotation, and assign it to the quaternion of the floating base.
		Eigen::Quaterniond q(Eigen::AngleAxisd(rng.uniformReal(-M_PI, M_PI), Eigen::Vector3d::UnitZ()));
		pos[3] = q.x();
		pos[4] = q.y();
		pos[5] = q.z();
		pos[6] = q.w();

		return moveit_facade::JointSpacePoint::from_moveit(state);
	}

	moveit_facade::JointSpacePoint robotStateFromPointAndArmvec(const moveit::core::RobotModel &drone,
																const Vec3d &desired_ee_pos,
																const Vec3d &armvec) {

		// Ensure armvec is normalized
		assert(std::abs(armvec.norm() - 1.0) < 1e-6);

		math::Polar polar = math::pointToPolar(armvec);

		Eigen::Quaterniond qd(Eigen::AngleAxisd(2.0 * M_PI - (M_PI / 2.0 - polar.azimuth), Eigen::Vector3d::UnitZ()));

		// Pick a base joint angle such that the arm is pointing in the right direction.
		moveit_facade::JointSpacePoint point(std::vector(drone.getVariableCount(), 0.0));

		point.joint_values[3] = qd.x();
		point.joint_values[4] = qd.y();
		point.joint_values[5] = qd.z();
		point.joint_values[6] = qd.w();

		moveEndEffectorToPoint(drone, point, desired_ee_pos);

		return point;

	}

	void moveEndEffectorNearPoint(const moveit::core::RobotModel &robot,
								  moveit_facade::JointSpacePoint &state,
								  const Vec3d &target,
								  double maxDistance) {

		// Get the end-effector position.
		math::Vec3d ee_pos = moveit_facade::computeEndEffectorPosition(robot, state);

		// Get the direction to the target.
		math::Vec3d delta = target - ee_pos;

		// Get the distance to the target.
		double distance = delta.norm();

		if (distance > maxDistance) {
			delta = delta.normalized() * (distance - maxDistance);

			// Apply.
			state.joint_values[0] += delta.x();
			state.joint_values[1] += delta.y();
			state.joint_values[2] += delta.z();
		}

		// else, we're already close enough, so do nothing.

	}

	moveit_facade::JointSpacePoint randomUprightWithBaseNearState(const moveit::core::RobotModel &robot,
																  double distance_bound,
																  const moveit_facade::JointSpacePoint &state,
																  const int seed) {

		random_numbers::RandomNumberGenerator rng(seed);

		moveit_facade::JointSpacePoint new_state = state;

		for (const moveit::core::JointModel* jm : robot.getJointModels()) {

			jm->getVariableRandomPositionsNearBy(rng, new_state.joint_values.data() + jm->getFirstVariableIndex(), state.joint_values.data() + jm->getFirstVariableIndex(),
			distance_bound);

		}

		// The base must be upright.
		// Compute a random yaw-rotation, and assign it to the quaternion of the floating base.
		Eigen::Quaterniond random_rotation(Eigen::AngleAxisd(rng.uniformReal(-distance_bound,distance_bound), Eigen::Vector3d::UnitZ()));
		Eigen::Quaterniond original_rotation(state.joint_values[6], state.joint_values[3], state.joint_values[4], state.joint_values[5]);
		// Compose the two.
		Eigen::Quaterniond new_rotation = random_rotation * original_rotation;
		// Assign the new rotation to the state.
		new_state.joint_values[3] = new_rotation.x();
		new_state.joint_values[4] = new_rotation.y();
		new_state.joint_values[5] = new_rotation.z();
		new_state.joint_values[6] = new_rotation.w();

		return new_state;

	}

	moveit_facade::JointSpacePoint genGoalSampleUniform(const Vec3d &target, int seed, const moveit::core::RobotModel &robot) {
		moveit_facade::JointSpacePoint jt = experiment_state_tools::randomUprightWithBase(robot, 0.0, seed);

		// Generate a point uniformly on a 3D sphere.

		random_numbers::RandomNumberGenerator rng(seed);
		rng.gaussian01();
		rng.gaussian01();
		rng.gaussian01();

		Vec3d random_point(rng.gaussian01(), rng.gaussian01(), rng.gaussian01());

		experiment_state_tools::moveEndEffectorNearPoint(
				robot, jt, target + random_point.normalized() * 0.05, 0.
		);
		return jt;
	}


}