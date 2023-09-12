// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 30-8-23.
//

#ifndef MGODPL_MYCOLLISIONENV_H
#define MGODPL_MYCOLLISIONENV_H


#include <moveit/collision_detection/collision_env.h>

class MyCollisionEnv : public collision_detection::CollisionEnv {

public:
	MyCollisionEnv() = delete;

	MyCollisionEnv(const moveit::core::RobotModelConstPtr& model, double padding = 0.0, double scale = 1.0);

	MyCollisionEnv(const moveit::core::RobotModelConstPtr& model, const collision_detection::WorldPtr& world, double padding = 0.0, double scale = 1.0);

	MyCollisionEnv(const MyCollisionEnv& other, const collision_detection::WorldPtr& world);

	~MyCollisionEnv() override;

	void checkSelfCollision(const collision_detection::CollisionRequest &req,
							collision_detection::CollisionResult &res,
							const moveit::core::RobotState &state) const override;

	void checkSelfCollision(const collision_detection::CollisionRequest &req,
							collision_detection::CollisionResult &res,
							const moveit::core::RobotState &state,
							const collision_detection::AllowedCollisionMatrix &acm) const override;

	void checkRobotCollision(const collision_detection::CollisionRequest &req,
							 collision_detection::CollisionResult &res,
							 const moveit::core::RobotState &state) const override;

	void checkRobotCollision(const collision_detection::CollisionRequest &req,
							 collision_detection::CollisionResult &res,
							 const moveit::core::RobotState &state,
							 const collision_detection::AllowedCollisionMatrix &acm) const override;

	void checkRobotCollision(const collision_detection::CollisionRequest &req,
							 collision_detection::CollisionResult &res,
							 const moveit::core::RobotState &state1,
							 const moveit::core::RobotState &state2,
							 const collision_detection::AllowedCollisionMatrix &acm) const override;

	void checkRobotCollision(const collision_detection::CollisionRequest &req,
							 collision_detection::CollisionResult &res,
							 const moveit::core::RobotState &state1,
							 const moveit::core::RobotState &state2) const override;

	void distanceSelf(const collision_detection::DistanceRequest &req,
					  collision_detection::DistanceResult &res,
					  const moveit::core::RobotState &state) const override;

	void distanceRobot(const collision_detection::DistanceRequest &req,
					   collision_detection::DistanceResult &res,
					   const moveit::core::RobotState &state) const override;
};


#endif //MGODPL_MYCOLLISIONENV_H
