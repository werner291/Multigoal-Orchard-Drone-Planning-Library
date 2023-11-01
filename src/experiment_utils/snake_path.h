// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/1/23.
//

#ifndef MGODPL_SNAKE_PATH_H
#define MGODPL_SNAKE_PATH_H

#include <functional>
#include "../math/Vec3.h"

namespace moveit::core {
	class RobotState;
}

namespace mgodpl::moveit_state_tools {

	using ParametricPath = std::function<math::Vec3d(double)>;

	moveit::core::RobotState put_on_path(const moveit::core::RobotState &state, const ParametricPath &path, double t);
}

#endif //MGODPL_SNAKE_PATH_H
