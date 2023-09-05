// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef MGODPL_MYCOLLISIONENV_H
#define MGODPL_MYCOLLISIONENV_H

#include <moveit/collision_detection/collision_env.h>
#include <moveit/collision_detection_fcl/collision_env_fcl.h>

class MyCollisionEnv : public collision_detection::CollisionEnvFCL {

public:

	void checkRobotCollision(const collision_detection::CollisionRequest &req,
							 collision_detection::CollisionResult &res,
							 const moveit::core::RobotState &state1,
							 const moveit::core::RobotState &state2,
							 const collision_detection::AllowedCollisionMatrix &acm) const override;

	void checkRobotCollision(const collision_detection::CollisionRequest &req,
							 collision_detection::CollisionResult &res,
							 const moveit::core::RobotState &state1,
							 const moveit::core::RobotState &state2) const override;

};


#endif //MGODPL_MYCOLLISIONENV_H
