// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 4/8/24.
//

#ifndef MGODPL_TRACING_H
#define MGODPL_TRACING_H

#include <vector>
#include "../math/Vec3.h"
#include "../planning/RobotModel.h"
#include "../planning/RobotPath.h"

namespace mgodpl {

	/**
	 * @brief This function generates a trace of a specific link's position along a robot path.
	 *
	 * @param robot		 The robot model.
	 * @param final_path The robot path.
	 * @param link 		 The ID of the link to trace.
	 * @return std::vector<math::Vec3d> A vector of 3D vectors representing the positions of the link along the path.
	 */
	std::vector<math::Vec3d> link_trace(const robot_model::RobotModel &robot,
										const RobotPath &final_path,
										const robot_model::RobotModel::LinkId link);

}

#endif //MGODPL_TRACING_H
