// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 12/4/23.
//

#include <vector>
#include "../math/Transform.h"
#include "RobotState.h"


mgodpl::RobotState mgodpl::interpolate(const mgodpl::RobotState &a, const mgodpl::RobotState &b, double t) {
	math::Transformd base_tf = interpolate(a.base_tf, b.base_tf, t);
	std::vector<double> joint_values;
	joint_values.reserve(a.joint_values.size());

	for (size_t i = 0; i < a.joint_values.size(); ++i) {
		joint_values.push_back(a.joint_values[i] * (1 - t) + b.joint_values[i] * t);
	}

	return {base_tf, joint_values};
}
