// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/22/24.
//

#include "goal_sampling.h"
#include "collision_detection.h"
#include "state_tools.h"

mgodpl::RobotState
mgodpl::genUprightState(const robot_model::RobotModel &robot, random_numbers::RandomNumberGenerator &rng) {

	// Initialize the fixed-size parts of the state.
	RobotState state = {
			.base_tf = math::Transformd{
					.translation = math::Vec3d(0.0, 0.0, 0.0), // At (0,0,0)
					.orientation = math::Quaterniond::fromAxisAngle(math::Vec3d::UnitZ(),
																	rng.uniformReal(-M_PI, M_PI)) // Random Z rotation/
			},
			.joint_values = {}
	};

	// Generate the arm joint variables.

	// Preallocate based on the number of joint variables.
	state.joint_values.reserve(robot.count_joint_variables());

	// Generate a value for each...
	for (const auto &joint: robot.getJoints()) {

		if (const auto &revolute = std::get_if<robot_model::RobotModel::RevoluteJoint>(&joint.type_specific)) {

			// If it's a revolute joint, generate a random value between the min and max angle.
			state.joint_values.push_back(rng.uniformReal(revolute->min_angle, revolute->max_angle));

		} else if (std::holds_alternative<robot_model::RobotModel::FixedJoint>(joint.type_specific)) {

			// Fixed joints do nothing and have no variables.

		} else {
			throw std::runtime_error("Unknown joint type");
		}
	}

	return state;
}

mgodpl::RobotState mgodpl::genGoalStateUniform(random_numbers::RandomNumberGenerator &rng,
											   const mgodpl::math::Vec3d &target,
											   double distance_from_target,
											   const mgodpl::robot_model::RobotModel &robot,
											   const mgodpl::robot_model::RobotModel::LinkId &flying_base,
											   const mgodpl::robot_model::RobotModel::LinkId &end_effector) {
	RobotState state = genUprightState(robot, rng);

	const auto &fk = robot_model::forwardKinematics(
			robot,
			state.joint_values,
			flying_base,
			state.base_tf
	);

	const auto &ee_tf = fk.forLink(end_effector);

	math::Vec3d target_delta = target - ee_tf.translation;

	// TODO: This should not be just a vector along the arm vector;
	// it'd make more sense to sample on the whole sphere.
	if (distance_from_target > 0.0) {
		target_delta += ee_tf.orientation.rotate(math::Vec3d(0.0, distance_from_target, 0.0));
	}

	state.base_tf.translation = state.base_tf.translation + target_delta;

	return state;
}

std::optional<mgodpl::RobotState> mgodpl::findGoalStateByUniformSampling(const mgodpl::math::Vec3d &target,
																		 const mgodpl::robot_model::RobotModel &robot,
																		 const mgodpl::robot_model::RobotModel::LinkId &
																		 flying_base,
																		 const mgodpl::robot_model::RobotModel::LinkId &
																		 end_effector,
																		 const fcl::CollisionObjectd &tree_trunk_object,
																		 random_numbers::RandomNumberGenerator &rng,
																		 size_t max_attempts) {
	for (size_t i = 0; i < max_attempts; ++i) {
		RobotState state = genGoalStateUniform(rng, target, robot, flying_base, end_effector);

		if (!check_robot_collision(robot, tree_trunk_object, state)) {
			return state;
		}
	}

	return std::nullopt;
}

std::optional<mgodpl::RobotState> mgodpl::generateUniformRandomArmVectorState(
		const robot_model::RobotModel &robot,
		const fcl::CollisionObjectd &tree_trunk_object,
		const math::Vec3d &fruit_center,
		random_numbers::RandomNumberGenerator &rng,
		const int max_attempts,
		const double ee_distance) {
	std::optional<RobotState> sample;

	for (int attempt = 0; attempt < max_attempts; ++attempt) {
		// Generate random arm vec.
		math::Vec3d arm_vec(rng.gaussian01(), rng.gaussian01(), rng.gaussian01());
		arm_vec = arm_vec.normalized();

		RobotState candidate = fromEndEffectorAndVector(robot, fruit_center - arm_vec * ee_distance, -arm_vec);

		if (!check_robot_collision(robot, tree_trunk_object, candidate)) {
			sample = candidate;
			break;
		}
	}

	return sample;
}
