#include <gtest/gtest.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include <moveit_msgs/PlanningScene.h>
#include "../src/experiment_utils.h"
#include "test_utils.h"

TEST(PTPTest, through_wall_local_adversarial) {

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