// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 12/4/23.
//

#pragma once

#include <vector>
#include "../math/Transform.h"

namespace mgodpl {
	struct RobotState {
		math::Transformd base_tf;
		std::vector<double> joint_values;

		[[nodiscard]] bool operator==(const RobotState &other) const {
			return base_tf == other.base_tf && joint_values == other.joint_values;
		}
	};

	/**
	 * @brief Compute the distance between the base link translations of two states.
	 *
	 * @param a 	The first state
	 * @param b		The second state
	 * @return 		The distance between the two base link translations
	 */
	double base_translation_distance(const RobotState &a, const RobotState &b);

	/**
	 * @brief Compute the sum of the distances between the base link translations and the base link orientations of two states.
	 *
	 * @param a					The first state
	 * @param b					The second state
	 * @param rotation_weight	The multiplier for the angular distance between the base orientations
	 * @return					The distance between the two states
	 */
	double base_distance(const RobotState &a, const RobotState &b, const double rotation_weight = 1.0);

	/**
	 * @brief Compute the distance between two states, assuming that all joint variables have equal weight.
	 *
	 * That is: it's the sum of:
	 * - The distance between the base transforms
	 * - The angular distance between the base orientations
	 * - The absolute difference between each joint variable
	 *
	 * @param a 	The first state
	 * @param b 	The second state
	 * @return 		The distance between the two states
	 */
	double equal_weights_distance(const RobotState &a, const RobotState &b);

	/**
	 * @brief Compute the distance between two states, assuming that all joint variables have equal weight.
	 * As opposed to equal_weights_distance, this function returns the maximum of the distances,
	 * modeling the idea that the robot's joints move independently.
	 *
	 * That is: it's the max of:
	 * - The distance between the base transforms
	 * - The angular distance between the base orientations
	 * - The absolute difference between each joint variable
	 *
	 * @param a 	The first state
	 * @param b 	The second state
	 * @return 		The distance between the two states
	 */
	double equal_weights_max_distance(const RobotState &a, const RobotState &b);

	RobotState interpolate(const RobotState &a, const RobotState &b, double t);
}
