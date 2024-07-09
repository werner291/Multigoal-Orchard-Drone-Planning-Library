// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/19/24.
//

#ifndef MGODPL_STATE_TOOLS_H
#define MGODPL_STATE_TOOLS_H

#include "RobotState.h"
#include "RobotModel.h"

namespace random_numbers {
	class RandomNumberGenerator;
}

namespace mgodpl {
	RobotState fromEndEffectorAndVector(
		const robot_model::RobotModel &robot,
		const math::Vec3d &endEffectorPoint,
		const math::Vec3d &vector
	);

	/**
	 *
	 * @brief Generate a random state, with the base translation in a [-range, range] x [0, range] x [-range, range] box.
	 *
	 * @param robot The robot model.
	 * @param rng	The random number generator.
	 * @param base_translation_range	The range in which the base translation is generated; this is a [-h_range, h_range] x [-h_range, h_range] x [0, v_range] box.
	 * @param joint_angle_range	The range in which the joint angles are generated; this is a [-range, range] box.
	 * @return	A random state.
	 */
	RobotState generateUniformRandomState(
		const robot_model::RobotModel &robot,
		random_numbers::RandomNumberGenerator &rng,
		const double h_range = 5.0,
		const double v_range = 10.0,
		const double joint_angle_range = M_PI / 2.0
	);
}

#endif //MGODPL_STATE_TOOLS_H
