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

		struct PlanState {
			moveit_facade::JointSpacePoint point;
			std::optional<math::Vec3d> target;
		};

		const double max_target_distance = 0.1;

	public:

		// Information about current upcoming plan.
		std::vector<PlanState> plan {};

		/**
		 * The list of fruit that is known about, may be reachable, and has not been visited yet.
		 */
		std::vector<math::Vec3d> fruit_to_visit {};

		explicit BlindlyMoveToNextFruit(moveit::core::RobotModelConstPtr  robot_model) : robot_model(std::move(robot_model)) {}

		std::optional<moveit_facade::JointSpacePoint> nextMovement(const mgodpl::experiments::VoxelShroudedSceneInfoUpdate &state) override;

		bool current_path_is_collision_free(const moveit_facade::JointSpacePoint& robot_current_state,
											const moveit_facade::CollisionDetection& collision_detection);

	};

}


#endif //MGODPL_BLINDLYMOVETONEXTFRUIT_H
