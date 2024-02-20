// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 2/20/24.
//

#ifndef MGODPL_SWEPT_VOLUME_CCD_H
#define MGODPL_SWEPT_VOLUME_CCD_H

#include <vector>
#include <array>
#include "RobotModel.h"
#include "RobotState.h"

namespace mgodpl {

	/**
	 * @brief Compute the swept volume of a robot moving from state1 to state2, represented as a list of triangles.
	 *
	 * @param robot 		The robot model.
	 * @param state1 		The initial state.
	 * @param state2 		The final state.
	 * @param segments 		The number of segments break the motion into; each segment uses a linear approximation.
	 * @return 				A list of triangles representing the swept volume.
	 */
	std::vector<std::array<mgodpl::math::Vec3d, 3>> swept_volume_triangles(const mgodpl::robot_model::RobotModel &robot,
																		   const mgodpl::RobotState &state1,
																		   const mgodpl::RobotState &state2,
																		   size_t segments);
}

#endif //MGODPL_SWEPT_VOLUME_CCD_H
