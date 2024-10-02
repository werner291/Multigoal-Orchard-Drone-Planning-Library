module;
// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 9/20/24.
//

#include <functional>
#include <optional>
#include "RobotModel.h"
#include "RobotState.h"


export module goal_sampling;

namespace mgodpl {

	/**
	 * Try a given operation at valid goal states, returning the return value of the first success.
	 *
	 * This function will sample at most 1000 states and check if they are valid goal states.
	 *
	 * For each that it finds, it will call the given operation with the goal state.
	 *
	 * If the operation succeeds, the result of that operation is returned.
	 *
	 * \param collision	 A function to check if a state collides.
	 * \param goal_sample A function to sample a goal state.
	 * \param max_samples The maximum number of samples to try.
	 * \param operation	 A function to call with a valid goal state.
	 *
	 * \returns The return value of `operation` if it was ever returned as not nullopt, nullopt otherwise
	 */
	export template<typename T>
	std::optional<T> try_at_valid_goal_samples(
			const std::function<bool(const RobotState &)> &collision,
			const std::function<RobotState()> &goal_sample,
			int max_samples,
			const std::function<std::optional<T>(const RobotState &)> &operation) {
		for (int sample_i = 0; sample_i < max_samples; ++sample_i) {
			RobotState goal_state = goal_sample();
			if (!collision(goal_state)) {
				if (auto path = operation(goal_state)) {
					return path;
				}
			}
		}
		return std::nullopt;
	}

	export RobotState project_to_goal(
			const robot_model::RobotModel& robot,
			RobotState to_project,
			const robot_model::RobotModel::LinkId flying_base,
			const robot_model::RobotModel::LinkId end_effector,
			const math::Vec3d target
			) {
		const auto &fk = robot_model::forwardKinematics(
				robot,
				to_project.joint_values,
				flying_base,
				to_project.base_tf
		);

		const auto &ee_tf = fk.forLink(end_effector);

		math::Vec3d target_delta = target - ee_tf.translation;

		to_project.base_tf.translation = to_project.base_tf.translation + target_delta;

		return to_project;
	}
}