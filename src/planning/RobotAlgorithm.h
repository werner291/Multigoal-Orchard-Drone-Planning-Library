// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/2/23.
//

#ifndef MGODPL_ROBOTALGORITHM_H
#define MGODPL_ROBOTALGORITHM_H

#include <vector>
#include <optional>
#include "../math/Vec3.h"
#include "JointSpacePoint.h"
#include "../experiment_utils/VoxelShroudedSceneInfo.h"

namespace mgodpl::planning {

	/**
	 * A movement-by-movement abstract algorithm class.
	 *
	 * The algorithm is given the current state of the robot, and the current state of the world,
	 * and must return the next state of the robot.
	 *
	 * This is in contrast to previous versions used in my PhD, where whole paths were put out.
	 */
	class RobotAlgorithm {

	public:

		/**
		 * The most important function of the algorithm. Given a state, return the next state.
		 * @param state
		 * @return
		 */
		virtual std::optional<moveit_facade::JointSpacePoint> nextMovement(const mgodpl::experiments::VoxelShroudedSceneInfoUpdate& state) = 0;

		virtual ~RobotAlgorithm() = default;

	};


}

#endif //MGODPL_ROBOTALGORITHM_H
