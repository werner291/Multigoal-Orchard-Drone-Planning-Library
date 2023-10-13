
#include <gtest/gtest.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include "test_utils.h"
#include "../src/utilities/experiment_utils.h"
#include "../src/BulletContinuousMotionValidator.h"

TEST(CollisionCheckingTests, max_angle) {

    auto robot = loadRobotModel();

    for (size_t i = 0; i < 1000; i++) {

        auto st1 = genRandomState(robot);
        auto st2 = genRandomState(robot);

        double max_angle = BulletContinuousMotionValidator::estimateMaximumRotation(*st1, *st2);

        for (const auto &lm: robot->getLinkModels()) {
            Eigen::Quaterniond r1(st1->getGlobalLinkTransform(lm).rotation());
            Eigen::Quaterniond r2(st2->getGlobalLinkTransform(lm).rotation());

            double angle = r1.angularDistance(r2);

            ASSERT_LE(angle, max_angle);
        }
    }
}

TEST(CollisionCheckingTests, symmetric_wall) {

    auto drone = loadRobotModel();

    ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");
    auto state_space = std::make_shared<DroneStateSpace>(spec);

    planning_scene::PlanningScenePtr scene = std::make_shared<planning_scene::PlanningScene>(drone);
    scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

    moveit_msgs::PlanningScene planning_scene_diff;

    spawn_wall(planning_scene_diff);

    scene->setPlanningSceneDiffMsg(planning_scene_diff);

    auto si = initSpaceInformation(scene, drone, state_space);

    ompl::base::ScopedState s1(si),s2(si);

    for (size_t i : boost::irange(0,1000)) {

        s1.random();
        s2.random();

        EXPECT_EQ(si->checkMotion(s1.get(),s2.get()),si->checkMotion(s2.get(),s1.get()));
    }
}