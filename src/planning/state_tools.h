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

namespace mgodpl {

	RobotState fromEndEffectorAndVector(
			const robot_model::RobotModel &robot,
			const math::Vec3d &endEffectorPoint,
			const math::Vec3d &vector
	);

}

#endif //MGODPL_STATE_TOOLS_H
