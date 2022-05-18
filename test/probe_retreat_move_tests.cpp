
#include <gtest/gtest.h>
#include <range/v3/range/conversion.hpp>
#include "../src/experiment_utils.h"
#include "../src/DronePathLengthObjective.h"
#include "../src/SphereShell.h"
#include "../src/ExperimentVisualTools.h"

TEST(ProbeRetreatMoveTests, state_outside_tree) {

    auto drone = loadRobotModel();

    auto [scene_msg, apples, SPHERE_CENTER, SPHERE_RADIUS] = createMeshBasedAppleTreePlanningSceneMessage("appletree");
    auto scene = setupPlanningScene(scene_msg, drone);

    const SphereShell sphereShell(SPHERE_CENTER, SPHERE_RADIUS);

    for (const Apple &apple: apples) {
        auto rs = sphereShell.state_on_shell(drone, apple);

        EXPECT_LT(
                (rs.getGlobalLinkTransform("end_effector").translation() -
                 sphereShell.applePositionOnShell(apple)).norm(),
                0.01
        );

        Eigen::Vector3d ee_ray = rs.getGlobalLinkTransform("end_effector").translation() - SPHERE_CENTER;
        Eigen::Vector3d base_ray = rs.getGlobalLinkTransform("base_link").translation() - SPHERE_CENTER;
        Eigen::Vector3d apple_ray = apple.center - SPHERE_CENTER;

        ee_ray.z() = 0.0;
        base_ray.z() = 0.0;
        apple_ray.z() = 0.0;

        EXPECT_NEAR(apple_ray.normalized().dot(ee_ray.normalized()), 1.0, 0.01);
        EXPECT_NEAR(apple_ray.normalized().dot(base_ray.normalized()), 1.0, 0.01);
        EXPECT_LT(ee_ray.norm() + 0.2, base_ray.norm());

        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        scene->checkCollision(req, res, rs);
        EXPECT_FALSE(res.collision);

    }

}


TEST(ProbeRetreatMoveTests, spherical_path) {

    auto drone = loadRobotModel();
    auto [scene_msg, apples, SPHERE_CENTER, SPHERE_RADIUS] = createMeshBasedAppleTreePlanningSceneMessage("appletree");

    const SphereShell sphereShell(SPHERE_CENTER, SPHERE_RADIUS);
    auto scene = setupPlanningScene(scene_msg, drone);

    for (const Apple &apple_a: apples) {
        for (const Apple &apple_b: apples) {

            if (apple_a.center == apple_b.center) continue;

            auto ra = sphereShell.state_on_shell(drone, apple_a);
            auto rb = sphereShell.state_on_shell(drone, apple_b);

            EXPECT_NEAR((ra.getGlobalLinkTransform("end_effector").translation() - SPHERE_CENTER).norm(),
                        (rb.getGlobalLinkTransform("end_effector").translation() - SPHERE_CENTER).norm(),
                        0.01);

            auto path = sphereShell.path_on_shell(drone, apple_a, apple_b);

            for (size_t i = 1; i < path.size(); ++i) {
                collision_detection::CollisionRequest req;
                req.verbose = true;
                collision_detection::CollisionResult res;
                scene->getCollisionEnv()->checkRobotCollision(req, res, path[i - 1], path[i]);
                EXPECT_FALSE(res.collision);
            }

            for (const auto &ri: path) {
                EXPECT_NEAR((ra.getGlobalLinkTransform("end_effector").translation() - SPHERE_CENTER).norm(),
                            SPHERE_RADIUS, 0.01);

                auto base_pos = ri.getGlobalLinkTransform("base_link").translation();

            }


        }
    }
}

TEST(ProbeRetreatMoveTests, cylinder_test) {

    auto drone = loadRobotModel();
    auto [scene_msg, apples, SPHERE_CENTER, SPHERE_RADIUS] = createMeshBasedAppleTreePlanningSceneMessage("appletree");

    ExperimentVisualTools evt;
    evt.publishPlanningScene(scene_msg);

    const CylinderShell shell(SPHERE_CENTER.topRows<2>(), SPHERE_RADIUS);
    auto scene = setupPlanningScene(scene_msg, drone);

    using namespace ranges;

    for (const Apple &apple_a: apples) {
        for (const Apple &apple_b: apples) {

            if (apple_a.center == apple_b.center) continue;

            auto ra = shell.state_on_shell(drone, apple_a);
            auto rb = shell.state_on_shell(drone, apple_b);

            EXPECT_NEAR((ra.getGlobalLinkTransform("end_effector").translation().topRows<2>() -
                         SPHERE_CENTER.topRows<2>()).norm(),
                        (rb.getGlobalLinkTransform("end_effector").translation().topRows<2>() -
                         SPHERE_CENTER.topRows<2>()).norm(),
                        0.01);

            auto path = shell.path_on_shell(drone, apple_a, apple_b);

            bool bad = false;
            for (size_t i = 1; i < path.size(); ++i) {
                collision_detection::CollisionRequest req;
                req.verbose = true;
                collision_detection::CollisionResult res;
                scene->getCollisionEnv()->checkRobotCollision(req, res, path[i - 1], path[i]);
                EXPECT_FALSE(res.collision);

                EXPECT_LT(path[i - 1].distance(path[i]), 0.5);

                bad |= res.collision;
            }

            if (bad) { // TODO remove

                robot_trajectory::RobotTrajectory traj(drone);
                for (const auto &item: path)
                    traj.addSuffixWayPoint(std::make_shared<moveit::core::RobotState>(item), 0.1);

                evt.publishTrajectory("test_traj", traj);

                ros::spin();

                std::cout << "Path:" << std::endl;

                for (const auto &ri: path) {

                    EXPECT_NEAR((ra.getGlobalLinkTransform("end_effector").translation().topRows<2>() -
                                 SPHERE_CENTER.topRows<2>()).norm(),
                                SPHERE_RADIUS, 0.01);

                    auto base_pos = ri.getGlobalLinkTransform("base_link").translation();

                    std::cout << base_pos.x() << "," << base_pos.y() << "," << base_pos.z() << "," << std::endl;

                }
            }


        }
    }


}

//
//TEST(ProbeRetreatMoveTests, optimize_exit) {
//
//    auto drone = loadRobotModel();
//
//    auto[scene_msg, apples, SPHERE_CENTER, SPHERE_RADIUS] = createMeshBasedAppleTreePlanningSceneMessage("appletree");
//
//    apples = apples | ranges::views::take(10) | ranges::to_vector;
//
//    auto state_space = std::make_shared<DroneStateSpace>(ompl_interface::ModelBasedStateSpaceSpecification(drone, "whole_body"));
//    auto si = initSpaceInformation(setupPlanningScene(scene_msg, drone), drone, state_space);
//    auto objective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);
//
//    const SphereShell sphereShell(SPHERE_CENTER, SPHERE_RADIUS);
//
//    OMPLSphereShellWrapper shell(sphereShell, si);
//
//    using namespace ranges;
//
//    auto approaches =
//            apples |
//            views::transform([&](auto apple) {
//                return planApproachesForApple(si, objective, apple, shell, [](auto si){
//                    return std::make_shared<ompl::geometric::RRTstar>(si);
//                });
//            }) | to_vector;
//
//    for (auto &[apple, approach] : approaches) {
//
//        moveit::core::RobotState rs(drone);
//
//        double cost_before = approach.length();
//
//        {
//            state_space->copyToRobotState(rs, approach.getState(approach.getStateCount() - 1));
//            EXPECT_LT((rs.getGlobalLinkTransform("end_effector").translation() - apple.center).norm(), 0.1);
//            state_space->copyToRobotState(rs, approach.getState(0));
//            Eigen::Vector3d ee_pos = rs.getGlobalLinkTransform("end_effector").translation();
//            EXPECT_NEAR((ee_pos - shell.getShell().getCenter()).norm(),shell.getShell().getRadius(), 0.1);
//        }
//        optimizeExit(apple, approach, objective, shell, si);
//
//        {
//            state_space->copyToRobotState(rs, approach.getState(approach.getStateCount() - 1));
//            EXPECT_LT((rs.getGlobalLinkTransform("end_effector").translation() - apple.center).norm(), 0.1);
//            state_space->copyToRobotState(rs, approach.getState(0));
//            Eigen::Vector3d ee_pos = rs.getGlobalLinkTransform("end_effector").translation();
//            EXPECT_NEAR((ee_pos - shell.getShell().getCenter()).norm(), shell.getShell().getRadius(), 0.1);
//        }
//
//        std::cout << "Before: " << cost_before << "After: " << approach.length() << std::endl;
//        EXPECT_LE(approach.length(), cost_before);
//    }
//
//}