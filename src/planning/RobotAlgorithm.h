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
#include "moveit_facade.h"

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

		struct ExternalStateUpdate {

			const moveit_facade::JointSpacePoint& current_state;
			const std::vector<math::Vec3d>& newly_detected_fruits;

		};

		/**
		 * The most important function of the algorithm. Given a state, return the next state.
		 * @param state
		 * @return
		 */
		virtual std::optional<moveit_facade::JointSpacePoint> nextMovement(const ExternalStateUpdate& state) = 0;
	};


}

#endif //MGODPL_ROBOTALGORITHM_H
