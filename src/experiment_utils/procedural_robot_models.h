// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef MGODPL_PROCEDURAL_ROBOT_MODELS_H
#define MGODPL_PROCEDURAL_ROBOT_MODELS_H

#include "../planning/RobotModel.h"

namespace mgodpl::experiments {
	const double STICK_LENGTH = 0.75;

	mgodpl::robot_model::RobotModel createProceduralRobotModel();

}

#endif //MGODPL_PROCEDURAL_ROBOT_MODELS_H
