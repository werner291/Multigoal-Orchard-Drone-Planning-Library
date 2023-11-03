// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef MGODPL_MYCOLLISIONENV_H
#define MGODPL_MYCOLLISIONENV_H

#include <moveit/collision_detection/collision_env.h>
#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include "moveit_forward_declarations.h"

class MyCollisionEnv : public collision_detection::CollisionEnvFCL {

public:
	MyCollisionEnv() = delete;

	MyCollisionEnv(const moveit::core::RobotModelConstPtr& model, const collision_detection::WorldPtr& world, double padding = 0.0, double scale = 1.0);

	struct ContinuousCollisionRequest {
		double distance_threshold;
	};

	struct ContinuousCollisionResult : public collision_detection::CollisionResult {
		double time_of_contact;
	};

	void checkRobotCollision(const ContinuousCollisionRequest &req,
							 ContinuousCollisionResult &res,
							 const moveit::core::RobotState &state1,
							 const moveit::core::RobotState &state2) const;



};


#endif //MGODPL_MYCOLLISIONENV_H
