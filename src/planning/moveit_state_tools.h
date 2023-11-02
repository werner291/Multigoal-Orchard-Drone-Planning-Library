// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/1/23.
//

#ifndef MGODPL_MOVEIT_STATE_TOOLS_H
#define MGODPL_MOVEIT_STATE_TOOLS_H

#include <memory>
#include "../math/Vec3.h"
#include "JointSpacePoint.h"

/**
 * A collection of utility functions that compute or modify robot states
 * in a way that makes sense in the context of the experiments.
 */
namespace mgodpl::experiment_state_tools {

	/**
	 * Translate the robot such that the end-effector is at the given target.
	 *
	 * @param robot 		The robot model.
	 * @param state 		The state to modify. (Mutated)
	 * @param target 		The target position.
	 */
	void moveEndEffectorToPoint(
			const moveit::core::RobotModel& robot,
			moveit_facade::JointSpacePoint &state,
			const math::Vec3d &target
			);

	/**
	 * Generate a state where the drone has an upright base, and is otherwise randomized.
	 *
	 * The translation of the base link is also randomized, but is bounded by the given value.
	 *
	 * Note: due to the lower bound on the z-coordinate, states will generally not be below the ground, but this is not guaranteed.
	 *
	 * @param robot				 The robot model.
	 * @param translation_bound	 The base link translation bound (to volume [-t,t] x [-t,t] x [0,t]).
	 * @param seed			     The seed for the random number generator.
	 * @return				     The generated state.
	 */
	moveit_facade::JointSpacePoint randomUprightWithBase(
			const moveit::core::RobotModel& robot,
			double translation_bound,
			const int seed
			);

	/**
	 * Generate a random state where the robot is positioned entirely outside the tree.
	 *
	 * We don't provide strong guarantees on distribution.
	 *
	 * @param robot 		The robot model.
	 * @param seed 			The seed for the random number generator.
	 * @return 				The generated state.
	 */
	moveit_facade::JointSpacePoint randomStateOutsideTree(
			const moveit::core::RobotModel& robot,
			int seed);

}

#endif //MGODPL_MOVEIT_STATE_TOOLS_H
