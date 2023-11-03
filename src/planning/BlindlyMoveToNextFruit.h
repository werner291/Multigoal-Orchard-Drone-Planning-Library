// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/2/23.
//

#ifndef MGODPL_BLINDLYMOVETONEXTFRUIT_H
#define MGODPL_BLINDLYMOVETONEXTFRUIT_H

#include <vector>
#include "../math/Vec3.h"
#include "RobotAlgorithm.h"
#include "moveit_forward_declarations.h"

namespace mgodpl::moveit_facade {
	class CollisionDetection;
}

namespace mgodpl::planning {

	/**
	 * A stupid algorithm that automatically moves until the end-effector of the robot
	 * is on the closest fruit.
	 */
	class BlindlyMoveToNextFruit : public RobotAlgorithm {

		const moveit::core::RobotModelConstPtr robot_model;

		std::vector<math::Vec3d> fruit_to_visit {};

	public:

		explicit BlindlyMoveToNextFruit(const moveit::core::RobotModelConstPtr& robot_model) : robot_model(robot_model) {}

		std::optional<moveit_facade::JointSpacePoint> nextMovement(const ExternalStateUpdate &state) override;

	};

}


#endif //MGODPL_BLINDLYMOVETONEXTFRUIT_H
