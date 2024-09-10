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

	RobotState interpolate(const RobotState &a, const RobotState &b, double t);
}
