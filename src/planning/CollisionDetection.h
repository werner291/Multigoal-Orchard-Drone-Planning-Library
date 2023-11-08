// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/2/23.
//

#ifndef MGODPL_COLLISIONDETECTION_H
#define MGODPL_COLLISIONDETECTION_H

#include <memory>
#include <shape_msgs/msg/mesh.hpp>
#include "JointSpacePoint.h"
#include "moveit_forward_declarations.h"
#include "JointSpacePath.h"

class MyCollisionEnv;

namespace mgodpl::moveit_facade {

	/**
     *  Class that wraps the collision detection functionality of MoveIt.
	 */
	class CollisionDetection {
		std::shared_ptr<MyCollisionEnv> collision_env = nullptr;

	public:
		explicit CollisionDetection(const std::vector<shape_msgs::msg::Mesh> &obstacle_meshes,
									const moveit::core::RobotModelConstPtr robot);

		[[nodiscard]] bool collides(const JointSpacePoint& state) const;

		[[nodiscard]] bool collides_ccd(const JointSpacePoint& state1, const JointSpacePoint& state2) const;

		/**
		 * Find the time of impact when moving between two states, if any.
		 *
		 * Effectively, assuming the robot moves from one state to the other, compute the time at which
		 * the robot first collides with an obstacle as an interpolation parameter between 0 and 1.
		 *
		 * Note: do not call collides_ccd() separately if you want to use the time of impact,
		 * as that function simply calls this function internally and discards the time of impact.
		 *
		 * @param state1 	The first state.
		 * @param state2 	The second state.
		 * @return 			The time of impact, or std::nullopt if there is no collision.
		 */
		[[nodiscard]] std::optional<double> collision_ccd_toi(const JointSpacePoint& state1, const JointSpacePoint& state2) const;

		bool path_collides(const JointSpacePath& path);
	};


}

#endif //MGODPL_COLLISIONDETECTION_H
