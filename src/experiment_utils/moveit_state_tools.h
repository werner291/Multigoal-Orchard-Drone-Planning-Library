// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/1/23.
//

#ifndef MGODPL_MOVEIT_STATE_TOOLS_H
#define MGODPL_MOVEIT_STATE_TOOLS_H

#include <memory>
#include "../math/Vec3.h"

namespace moveit::core {
	class RobotModel;
	using RobotModelConstPtr = std::shared_ptr<const RobotModel>;
	class RobotState;
}

namespace mgodpl::experiments {

	void moveEndEffectorToGoal(moveit::core::RobotState &state, double tolerance, const math::Vec3d &target);

	void randomizeUprightWithBase(moveit::core::RobotState &state, double translation_bound);

	moveit::core::RobotState randomStateOutsideTree(const moveit::core::RobotModelConstPtr &drone, const int seed);
}

#endif //MGODPL_MOVEIT_STATE_TOOLS_H
