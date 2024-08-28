// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 8/28/24.
//

#ifndef ROBOTPATHFN_H
#define ROBOTPATHFN_H
#include <functional>

#include "RobotState.h"

namespace mgodpl {

	using RobotPathFn = std::function<RobotState(double)>;

	RobotPathFn slice(RobotPathFn path_function, double start, double end);

	RobotPathFn concat(std::initializer_list<RobotPathFn> path_functions);

} // namespace mgodpl

#endif // ROBOTPATHFN_H
