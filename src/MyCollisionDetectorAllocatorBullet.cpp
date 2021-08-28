//
// Created by werner on 8/27/21.
//

#include "MyCollisionDetectorAllocatorBullet.h"


const std::string MyCollisionDetectorAllocatorBullet::NAME = "MyCollisionDetectorAllocatorBullet";

MyCollisionEnvironment::MyCollisionEnvironment(const moveit::core::RobotModelConstPtr &model, double padding,
                                               double scale) : CollisionEnvBullet(model, padding, scale) {}

MyCollisionEnvironment::MyCollisionEnvironment(const collision_detection::CollisionEnvBullet &other,
                                               const collision_detection::WorldPtr &world) : CollisionEnvBullet(other,
                                                                                                                world) {}

MyCollisionEnvironment::MyCollisionEnvironment(const moveit::core::RobotModelConstPtr &model,
                                               const collision_detection::WorldPtr &world, double padding, double scale)
        : CollisionEnvBullet(model, world, padding, scale) {}