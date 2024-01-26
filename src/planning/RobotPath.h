// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/22/24.
//

#ifndef MGODPL_ROBOTPATH_H
#define MGODPL_ROBOTPATH_H


#include <vector>
#include "RobotState.h"

namespace mgodpl {
	struct RobotPath {
		std::vector<RobotState> states;
	};
}


#endif //MGODPL_ROBOTPATH_H
