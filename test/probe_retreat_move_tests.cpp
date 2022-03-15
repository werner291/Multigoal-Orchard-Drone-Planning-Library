
#include <gtest/gtest.h>
#include "../src/experiment_utils.h"
#include "../src/probe_retreat_move.h"

TEST(ProbeRetreatMoveTests, state_outside_tree) {

    auto drone = loadRobotModel();

    auto[scene_msg, apples] = createMeshBasedAppleTreePlanningSceneMessage();

    Eigen::Vector3d sphere_center(0.0,0.0,2.0);

    for (const Apple &apple: apples) {
        auto rs = state_outside_tree(drone, apple, sphere_center, 4.0);

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

    Eigen::Vector3d sphere_center(0.0,0.0,2.0);

    auto ra = state_outside_tree(drone, {Eigen::Vector3d(0.5, 0.2, 2.2), Eigen::Vector3d(0.0, 0.0, 0.0)}, sphere_center,
                                 4.0);
    auto rb = state_outside_tree(drone, {Eigen::Vector3d(0.33, -1.2, 2.2), Eigen::Vector3d(0.0, 0.0, 0.0)},
                                 sphere_center, 4.0);

    EXPECT_NEAR((ra.getGlobalLinkTransform("end_effector").translation() - sphere_center).norm(),
                (rb.getGlobalLinkTransform("end_effector").translation() - sphere_center).norm(),
                0.01);

    auto path = sphericalInterpolatedPath(ra, rb, sphere_center);

    for (const auto &ri : path) {
        EXPECT_GT((ra.getGlobalLinkTransform("end_effector").translation() - sphere_center).norm(), 2.5);
    }

}