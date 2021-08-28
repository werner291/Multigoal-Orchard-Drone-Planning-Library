//
// Created by werner on 8/27/21.
//

#ifndef NEW_PLANNERS_MYCOLLISIONDETECTORALLOCATORBULLET_H
#define NEW_PLANNERS_MYCOLLISIONDETECTORALLOCATORBULLET_H

#include <moveit/collision_detection/collision_detector_allocator.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>

class MyCollisionEnvironment : public collision_detection::CollisionEnvBullet {
public:
    MyCollisionEnvironment(const moveit::core::RobotModelConstPtr &model, double padding = 0.0, double scale = 1.0);

    MyCollisionEnvironment(const CollisionEnvBullet &other, const collision_detection::WorldPtr &world);

    MyCollisionEnvironment(const moveit::core::RobotModelConstPtr &model, const collision_detection::WorldPtr &world,
                           double padding = 0.0, double scale = 1.0);

    void robotToEnvironmentClearance(const moveit::core::RobotState &state) const;
};

class MyCollisionDetectorAllocatorBullet : public collision_detection::CollisionDetectorAllocatorTemplate<MyCollisionEnvironment, MyCollisionDetectorAllocatorBullet> {

public:
    static const std::string NAME;

};




#endif //NEW_PLANNERS_MYCOLLISIONDETECTORALLOCATORBULLET_H
