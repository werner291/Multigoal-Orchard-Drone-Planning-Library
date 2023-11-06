// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/2/23.
//

#ifndef MGODPL_BLINDLYMOVETONEXTFRUIT_H
#define MGODPL_BLINDLYMOVETONEXTFRUIT_H

#include <utility>
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



		struct NextTargetPlan {
			std::vector<moveit_facade::JointSpacePoint> path;
			math::Vec3d target;
		};

	public:

		// Information about current behavior.
		std::optional<NextTargetPlan> plan = std::nullopt;

		/**
		 * The list of fruit that is known about, may be reachable, and has not been visited yet.
		 */
		std::vector<math::Vec3d> fruit_to_visit {};

		explicit BlindlyMoveToNextFruit(moveit::core::RobotModelConstPtr  robot_model) : robot_model(std::move(robot_model)) {}

		std::optional<moveit_facade::JointSpacePoint> nextMovement(const mgodpl::experiments::VoxelShroudedSceneInfoUpdate &state) override;

	};

}


#endif //MGODPL_BLINDLYMOVETONEXTFRUIT_H
