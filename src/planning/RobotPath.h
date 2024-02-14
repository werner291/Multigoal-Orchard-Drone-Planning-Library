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

	/**
	 * A point on a path if such a path is defined based on states and the interpolated motions between them.
	 */
	struct PathPoint {
		/// The segment of the path between states segment_i and segment_i+1 (0-indexed)
		size_t segment_i;
		///
		double segment_t;
	};
}


#endif //MGODPL_ROBOTPATH_H
