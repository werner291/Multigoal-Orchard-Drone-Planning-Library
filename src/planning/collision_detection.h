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
#include "RobotPath.h"

namespace mgodpl {
	/**
	 * Check for collisions of a single robot link with a given fcl collision object.
	 *
	 * @param link 					The link to check.
	 * @param tree_trunk_object 	The collision object to check against.
	 * @param link_tf 				The transform of the link.
	 * @return 						True if there is a collision, false otherwise.
	 */
	bool check_link_collision(const robot_model::RobotModel::Link &link,
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
	bool check_robot_collision(const robot_model::RobotModel &robot,
	                           const fcl::CollisionObjectd &tree_trunk_object,
	                           const RobotState &state);

	/**
	 * Check whether a motion from one state to another (assuming linear interpolation in the configuration space)
	 * collides with the given CollisionObjectd.
	 *
	 * TODO: This function uses samples, and is not a full CCD function.
	 *
	 * @param robot 				The robot model.
	 * @param tree_trunk_object 	The tree trunk that the robot must not collide with.
	 * @param state1 				The state to start from.
	 * @param state2 				The state to end at.
	 * @param toi 					The linear interpolation parameter of the last known state to not collide. (Undefined if the function returns false.)
	 * @return 						True if the motion collides, false otherwise.
	 */
	bool check_motion_collides(const robot_model::RobotModel &robot,
	                           const fcl::CollisionObjectd &tree_trunk_object,
	                           const RobotState &state1,
	                           const RobotState &state2,
	                           double &toi);

	/**
	 * Check whether a motion from one state to another (assuming linear interpolation in the configuration space)
	 * collides with the given CollisionObjectd.
	 *
	 * TODO: This function uses samples, and is not a full CCD function.
	 *
	 * @param robot 				The robot model.
	 * @param tree_trunk_object 	The tree trunk that the robot must not collide with.
	 * @param state1 				The state to start from.
	 * @param state2 				The state to end at.
	 * @return 						True if the motion collides, false otherwise.
	 */
	inline bool check_motion_collides(const mgodpl::robot_model::RobotModel &robot,
	                                  const fcl::CollisionObjectd &tree_trunk_object,
	                                  const mgodpl::RobotState &state1,
	                                  const mgodpl::RobotState &state2) {
		double dummy_toi; // This will act as a placeholder for the 'time of impact' parameter.
		return check_motion_collides(robot, tree_trunk_object, state1, state2, dummy_toi);
	}

	/**
	 * Check whether the given path collides.
	 *
	 * @param robot 				The robot model.
	 * @param tree_trunk_object		The tree trunk that the robot must not collide with.
	 * @param path					The path to check for collisions.
	 * @param collision_point		Return param for the point along the path that was last-known to be valid.
	 * @return						True if the path collides, false otherwise.
	 */
	bool check_path_collides(const robot_model::RobotModel &robot,
	                         const fcl::CollisionObjectd &tree_trunk_object,
	                         const RobotPath &path,
	                         PathPoint &collision_point);
}

#endif //MGODPL_COLLISION_DETECTION_H
