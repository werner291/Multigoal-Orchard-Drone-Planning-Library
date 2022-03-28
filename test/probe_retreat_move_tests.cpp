
#include <gtest/gtest.h>
#include <ompl/geometric/PathSimplifier.h>
#include <range/v3/view/take.hpp>
#include <range/v3/range/conversion.hpp>
#include "../src/experiment_utils.h"
#include "../src/probe_retreat_move.h"
#include "../src/ManipulatorDroneMoveitPathLengthObjective.h"

#include <range/v3/all.hpp>

TEST(ProbeRetreatMoveTests, state_outside_tree) {

    auto drone = loadRobotModel();

    auto[scene_msg, apples] = createMeshBasedAppleTreePlanningSceneMessage();

    Eigen::Vector3d sphere_center(0.0,0.0,2.2);

    const SphereShell sphereShell(sphere_center, 4.0);


    for (const Apple &apple: apples) {
        auto rs = sphereShell.state_on_shell(drone, apple);

        Eigen::Vector3d ee_ray = rs.getGlobalLinkTransform("end_effector").translation() - sphere_center;
        Eigen::Vector3d base_ray = rs.getGlobalLinkTransform("base_link").translation() - sphere_center;
        Eigen::Vector3d apple_ray = apple.center - sphere_center;

        ee_ray.z() = 0.0;
        base_ray.z() = 0.0;
        apple_ray.z() = 0.0;

        EXPECT_NEAR(apple_ray.normalized().dot(ee_ray.normalized()), 1.0, 0.01);
        EXPECT_NEAR(apple_ray.normalized().dot(base_ray.normalized()), 1.0, 0.01);
        EXPECT_LT(ee_ray.norm(), base_ray.norm());

    }

}


TEST(ProbeRetreatMoveTests, spherical_path) {

    auto drone = loadRobotModel();

    Eigen::Vector3d sphere_center(0.0,0.0,2.02);

    const SphereShell sphereShell(sphere_center, 4.0);

    Apple apple_a {Eigen::Vector3d(0.5, 0.2, 2.2), Eigen::Vector3d(0.0, 0.0, 0.0)};
    Apple apple_b {Eigen::Vector3d(0.33, -1.2, 2.2), Eigen::Vector3d(0.0, 0.0, 0.0)};

    auto ra = sphereShell.state_on_shell(drone, apple_a);
    auto rb = sphereShell.state_on_shell(drone, apple_b);

    EXPECT_NEAR((ra.getGlobalLinkTransform("end_effector").translation() - sphere_center).norm(),
                (rb.getGlobalLinkTransform("end_effector").translation() - sphere_center).norm(),
                0.01);

    auto path = sphereShell.path_on_shell(drone, apple_a, apple_b);

    for (const auto &ri : path) {
        EXPECT_GT((ra.getGlobalLinkTransform("end_effector").translation() - sphere_center).norm(), 2.5);
    }

}

TEST(ProbeRetreatMoveTests, optimize_exit) {

    auto drone = loadRobotModel();

    auto[scene_msg, apples] = createMeshBasedAppleTreePlanningSceneMessage();

    apples = apples | ranges::views::take(10) | ranges::to_vector;

    auto state_space = std::make_shared<DroneStateSpace>(ompl_interface::ModelBasedStateSpaceSpecification(drone, "whole_body"));
    auto si = initSpaceInformation(setupPlanningScene(scene_msg, drone), drone, state_space);
    auto objective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);
    const Eigen::Vector3d sphere_center(0.0,0.0,2.2);
    const SphereShell sphereShell(sphere_center, 1.9);

    OMPLSphereShellWrapper shell(sphereShell, si);
    auto approaches = planApproaches(apples, objective, shell, si);

    for (auto &[apple, approach] : approaches) {

        moveit::core::RobotState rs(drone);

        double cost_before = approach.length();

        {
            state_space->copyToRobotState(rs, approach.getState(approach.getStateCount() - 1));
            EXPECT_LT((rs.getGlobalLinkTransform("end_effector").translation() - apple.center).norm(), 0.1);
            state_space->copyToRobotState(rs, approach.getState(0));
            Eigen::Vector3d ee_pos = rs.getGlobalLinkTransform("end_effector").translation();
            EXPECT_NEAR((ee_pos - shell.getShell().getCenter()).norm(),shell.getShell().getRadius(), 0.1);
        }
        optimizeExit(apple, approach, objective, shell, si);

        {
            state_space->copyToRobotState(rs, approach.getState(approach.getStateCount() - 1));
            EXPECT_LT((rs.getGlobalLinkTransform("end_effector").translation() - apple.center).norm(), 0.1);
            state_space->copyToRobotState(rs, approach.getState(0));
            Eigen::Vector3d ee_pos = rs.getGlobalLinkTransform("end_effector").translation();
            EXPECT_NEAR((ee_pos - shell.getShell().getCenter()).norm(), shell.getShell().getRadius(), 0.1);
        }

        std::cout << "Before: " << cost_before << "After: " << approach.length() << std::endl;
        EXPECT_LE(approach.length(), cost_before);
    }

}