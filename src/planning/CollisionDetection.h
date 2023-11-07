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

		bool collides(const JointSpacePoint& state) const;

		bool collides_ccd(const JointSpacePoint& state1, const JointSpacePoint& state2) const;

		bool path_collides(const JointSpacePath& path);
	};


}

#endif //MGODPL_COLLISIONDETECTION_H
