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
	 * Generate an upright robot state, without checking for collisions.
	 * @param rng 		The random number generator to use.
	 * @return 			The generated robot state.
	 */
	RobotState genUprightState(random_numbers::RandomNumberGenerator &rng);

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
		const robot_model::RobotModel &robot,
		const robot_model::RobotModel::LinkId &flying_base,
		const robot_model::RobotModel::LinkId &end_effector);

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
