// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/1/23.
//

#include "snake_path.h"

#include <moveit/robot_state/robot_state.h>

namespace mgodpl::moveit_state_tools {

	moveit::core::RobotState put_on_path(const moveit::core::RobotState &state, const ParametricPath &path, double t) {
		throw std::runtime_error("Not implemented yet!");
	}
}