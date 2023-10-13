// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 26-6-23.
//

#ifndef MGODPL_MOVEITCONFIGURATIONSPACE_H
#define MGODPL_MOVEITCONFIGURATIONSPACE_H

#include <moveit/robot_state/robot_state.h>

namespace mgodpl::moveit_impl {

	struct RobotConfigurationSpace {
		using Configuration = moveit::core::RobotState;
	};

}

#endif //MGODPL_MOVEITCONFIGURATIONSPACE_H
