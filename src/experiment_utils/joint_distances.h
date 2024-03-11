// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/11/24.
//

#ifndef MGODPL_JOINT_DISTANCES_H
#define MGODPL_JOINT_DISTANCES_H

#include <json/value.h>
#include "../planning/RobotState.h"

namespace mgodpl {

	/**
	 * A struct of the distances between two robot states on a per-joint basis.
	 */
	struct JointDistances {
		/// The distance between the base translations
		double translation_distance;
		/// The distance between the base rotations (in ra
		double rotation_distance;
		/// The distance between the angular positions of each joint (absolute value)
		std::vector<double> joint_distances;
	};

	/**
	 * Calculate the distances between two robot states.
	 * @param state1 	The first robot state.
	 * @param state2 	The second robot state.
	 * @return A struct containing the distances between the two states.
	 */
	[[nodiscard]] JointDistances calculateJointDistances(const RobotState &state1, const RobotState &state2);

	[[nodiscard]] Json::Value toJson(const JointDistances &distances);
}

#endif //MGODPL_JOINT_DISTANCES_H
