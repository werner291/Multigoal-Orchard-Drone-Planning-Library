// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 30-8-23.
//

#include "MyCollisionEnv.h"

void MyCollisionEnv::checkSelfCollision(const collision_detection::CollisionRequest &req,
										collision_detection::CollisionResult &res,
										const moveit::core::RobotState &state) const {

}

void MyCollisionEnv::checkSelfCollision(const collision_detection::CollisionRequest &req,
										collision_detection::CollisionResult &res,
										const moveit::core::RobotState &state,
										const collision_detection::AllowedCollisionMatrix &acm) const {

}

void MyCollisionEnv::checkRobotCollision(const collision_detection::CollisionRequest &req,
										 collision_detection::CollisionResult &res,
										 const moveit::core::RobotState &state) const {

}

void MyCollisionEnv::checkRobotCollision(const collision_detection::CollisionRequest &req,
										 collision_detection::CollisionResult &res,
										 const moveit::core::RobotState &state,
										 const collision_detection::AllowedCollisionMatrix &acm) const {

}

void MyCollisionEnv::checkRobotCollision(const collision_detection::CollisionRequest &req,
										 collision_detection::CollisionResult &res,
										 const moveit::core::RobotState &state1,
										 const moveit::core::RobotState &state2,
										 const collision_detection::AllowedCollisionMatrix &acm) const {

}

void MyCollisionEnv::checkRobotCollision(const collision_detection::CollisionRequest &req,
										 collision_detection::CollisionResult &res,
										 const moveit::core::RobotState &state1,
										 const moveit::core::RobotState &state2) const {

}

void MyCollisionEnv::distanceSelf(const collision_detection::DistanceRequest &req,
								  collision_detection::DistanceResult &res,
								  const moveit::core::RobotState &state) const {

}

void MyCollisionEnv::distanceRobot(const collision_detection::DistanceRequest &req,
								   collision_detection::DistanceResult &res,
								   const moveit::core::RobotState &state) const {

}

MyCollisionEnv::MyCollisionEnv(const moveit::core::RobotModelConstPtr &model, double padding, double scale) {
	MyCollisionEnv(model, nullptr, padding, scale);
}

MyCollisionEnv::MyCollisionEnv(const moveit::core::RobotModelConstPtr &model,
							   const collision_detection::WorldPtr &world,
							   double padding,
							   double scale) {

}

MyCollisionEnv::MyCollisionEnv(const MyCollisionEnv &other, const collision_detection::WorldPtr &world) {
	throw std::runtime_error("Not implemented");
}
