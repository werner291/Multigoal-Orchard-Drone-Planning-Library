// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/22/24.
//

#ifndef MGODPL_GOAL_SAMPLING_H
#define MGODPL_GOAL_SAMPLING_H

#include <optional>
#include "RobotState.h"
#include "RobotModel.h"
#include "fcl_forward_declarations.h"
#include "RandomNumberGenerator.h"

namespace mgodpl {
	/**
	 * @brief Generates an upright state for a given robot model, without collision checking.
	 *
	 * This function generates an upright state for a given robot model.
	 *
	 * The state consists of a base transform and joint values.
	 * The base transform is initialized with a translation of (0.0, 0.0, 0.0) and a random orientation around the Z-axis.
	 *
	 * The joint values are initialized based on the joint types of the robot model.
	 * - For revolute joints, a random value between the minimum and maximum angle is chosen.
	 * - For fixed joints, no value is added.
	 * If an unknown joint type is encountered, the function throws a runtime error.
	 *
	 * @param robot The robot model for which to generate an upright state.
	 * @param rng A random number generator used to generate the random orientation and joint values.
	 * @return The generated upright state for the robot model.
	 */
	RobotState genUprightState(const robot_model::RobotModel &robot, random_numbers::RandomNumberGenerator &rng);

	/**
	 * Generate a state where the end effector is at the given target, not checking for collisions.
	 *
	 * @param rng 					The random number generator to use.
	 * @param target 				The target position.
	 * @param robot 				The robot model.
	 * @param flying_base 			The link ID of the flying base.
	 * @param end_effector 			The link ID of the end effector.
	 * @return 						The generated robot state.
	 */
	RobotState genGoalStateUniform(
			random_numbers::RandomNumberGenerator &rng,
			const math::Vec3d &target,
			double distance_from_target,
			const robot_model::RobotModel &robot,
			const robot_model::RobotModel::LinkId &flying_base,
			const robot_model::RobotModel::LinkId &end_effector);

	/**
	 * Generate a state where the end effector is at the given target, not checking for collisions.
	 *
	 * @param rng 					The random number generator to use.
	 * @param target 				The target position.
	 * @param robot 				The robot model.
	 * @param flying_base 			The link ID of the flying base.
	 * @param end_effector 			The link ID of the end effector.
	 * @return 						The generated robot state.
	 */
	inline RobotState genGoalStateUniform(
			random_numbers::RandomNumberGenerator &rng,
			const math::Vec3d &target,
			const robot_model::RobotModel &robot,
			const robot_model::RobotModel::LinkId &flying_base,
			const robot_model::RobotModel::LinkId &end_effector) {
		return genGoalStateUniform(rng, target, 0.0, robot, flying_base, end_effector);
	}


	/**
	 * Attempt to find a collision-free goal state by uniform sampling.
	 *
	 * @param target 					The target position that the end-effector should be near.
	 * @param robot 					The robot model.
	 * @param flying_base 				The link ID of the flying base.
	 * @param end_effector 				The link ID of the end effector.
	 * @param tree_trunk_object 		The collision object of the tree trunk.
	 * @param rng 						The random number generator to use.
	 * @param max_attempts 				The maximum number of attempts to make.
	 * @return 							The generated robot state, or nullopt if no state was found.
	 */
	std::optional<RobotState> findGoalStateByUniformSampling(
			const math::Vec3d &target,
			const robot_model::RobotModel &robot,
			const robot_model::RobotModel::LinkId &flying_base,
			const robot_model::RobotModel::LinkId &end_effector,
			const fcl::CollisionObjectd &tree_trunk_object,
			random_numbers::RandomNumberGenerator &rng,
			size_t max_attempts);

	/**
	 * @brief Attempts to generate a collision-free RobotState by uniformly sampling random arm vectors.
	 *
	 * @param robot					The robot model.
	 * @param tree_trunk_object		The collision object of the tree trunk.
	 * @param fruit_center			The center of the fruit.
	 * @param rng The random number generator to use.
	 * @param max_attempts The maximum number of attempts to make.
	 * @param ee_distance The distance from the end effector to the center of the fruit.
	 * @return A collision-free RobotState, or std::nullopt if no state was found.
	 */
	std::optional<RobotState> generateUniformRandomArmVectorState(
			const robot_model::RobotModel &robot,
			const fcl::CollisionObjectd &tree_trunk_object,
			const math::Vec3d &fruit_center,
			random_numbers::RandomNumberGenerator &rng,
			int max_attempts,
			double ee_distance);
}

#endif //MGODPL_GOAL_SAMPLING_H
