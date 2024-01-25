// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/15/24.
//

#ifndef MGODPL_COLLISION_DETECTION_H
#define MGODPL_COLLISION_DETECTION_H

#include <vector>
#include "fcl_forward_declarations.h"
#include "../math/Transform.h"
#include "RobotModel.h"
#include "RobotState.h"

namespace mgodpl {

	/**
	 * Check for collisions of a single robot link with a given fcl collision object.
	 *
	 * @param link 					The link to check.
	 * @param tree_trunk_object 	The collision object to check against.
	 * @param link_tf 				The transform of the link.
	 * @return 						True if there is a collision, false otherwise.
	 */
	bool check_link_collision(const mgodpl::robot_model::RobotModel::Link &link,
							  const fcl::CollisionObjectd &tree_trunk_object,
							  const math::Transformd &link_tf);

	/**
	 * Check for collisions between a whole robot and a single FCL collision object.
	 *
	 * @param robot 					The robot to check.
	 * @param tree_trunk_object 		The collision object to check against.
	 * @param state 					The state of the robot.
	 * @return 							True if there is a collision, false otherwise.
	 */
	bool check_robot_collision(const mgodpl::robot_model::RobotModel &robot,
							   const fcl::CollisionObjectd &tree_trunk_object,
							   const mgodpl::RobotState &state);
}

#endif //MGODPL_COLLISION_DETECTION_H
