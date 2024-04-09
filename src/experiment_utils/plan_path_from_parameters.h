// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 4/8/24.
//

#ifndef MGODPL_PLAN_PATH_FROM_PARAMETERS_H
#define MGODPL_PLAN_PATH_FROM_PARAMETERS_H

#include "../planning/RobotPath.h"
#include "declarative/SolutionMethod.h"
#include "declarative_environment.h"

namespace mgodpl {

	/**
	 * Plan a path in an environment based on declarative parameters.
	 *
	 * @param method 		A specification of the method to use.
	 * @param env 			The environment in which to plan the path (see create_environment).
	 * @return 				The planned path.
	 */
	RobotPath plan_multigoal_path_in_scenario(const declarative::SolutionMethod& method, const declarative::PointScanEnvironment& env);
}

#endif //MGODPL_PLAN_PATH_FROM_PARAMETERS_H
