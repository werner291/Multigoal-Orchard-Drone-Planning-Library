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

// We forward declare the CollisionEnv class from Movet so we don't deal with the MoveIt headers
// (and their inclusion of Eigen) everywhere we import this header.
namespace collision_detection {
	class CollisionEnv;
}

namespace mgodpl::moveit_facade {

	/**
     *  Class that wraps the collision detection functionality of MoveIt.
	 */
	class CollisionDetection {
		std::shared_ptr<collision_detection::CollisionEnv> collision_env = nullptr;

	public:
		explicit CollisionDetection(const std::vector<shape_msgs::msg::Mesh> &obstacle_meshes,
									const moveit::core::RobotModelConstPtr robot);

		bool collides(const JointSpacePoint& state);

	private:

		bool collides_ccd(const JointSpacePoint& state1, const JointSpacePoint& state2);
	};

}

#endif //MGODPL_COLLISIONDETECTION_H
