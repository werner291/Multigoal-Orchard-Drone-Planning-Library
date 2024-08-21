// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 4/8/24.
//

#ifndef MGODPL_LADDER_TRACE_H
#define MGODPL_LADDER_TRACE_H

#include "../planning/RobotModel.h"
#include "../planning/RobotPath.h"
#include "SimpleVtkViewer.h"

namespace mgodpl::visualization {

	void visualize_ladder_trace(const mgodpl::robot_model::RobotModel &robot,
								const mgodpl::RobotPath &final_path,
								SimpleVtkViewer &viewer);
}

#endif //MGODPL_LADDER_TRACE_H
