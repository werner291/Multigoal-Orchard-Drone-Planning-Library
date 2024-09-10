// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10-9-24.
//

#ifndef MGODPL_ROUGHNESS_H
#define MGODPL_ROUGHNESS_H

#include <functional>
#include "RobotState.h"
#include "distance.h"

namespace mgodpl {

	using RoughnessFn = std::function<double(const RobotState &, const RobotState &, const RobotState &)>;

	RoughnessFn distance_ratio_roughness(const DistanceFn &distance_fn);

	RoughnessFn distance_ratio_roughness(const DistanceFn &distance_fn) {
		return [distance_fn](const RobotState &a, const RobotState &b, const RobotState &c) {
			return (distance_fn(a, b) + distance_fn(b, c)) / distance_fn(a, c);
		};
	}

}

#endif //MGODPL_ROUGHNESS_H
