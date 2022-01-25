#include "../src/BulletContinuousMotionValidator.h"
#include <gtest/gtest.h>
#include "test_utils.h"
#include "../src/experiment_utils.h"
#include <eigen_conversions/eigen_msg.h>

std::shared_ptr<moveit::core::RobotState> genRandomState(const std::shared_ptr<moveit::core::RobotModel> &drone) {
    auto st1 = std::make_shared<moveit::core::RobotState>(drone);
    st1->setToRandomPositions();
    double *st1_values = st1->getVariablePositions();
    st1_values[0] = st1->getRandomNumberGenerator().uniformReal(-100.0, 100.0);
    st1_values[1] = st1->getRandomNumberGenerator().uniformReal(-100.0, 100.0);
    st1_values[2] = st1->getRandomNumberGenerator().uniformReal(-100.0, 100.0);
    return st1;
}

std::vector<std::shared_ptr<ompl::base::GoalSampleableRegion>> genGoals(const ompl::base::SpaceInformationPtr &si) {
    std::vector<std::shared_ptr<ompl::base::GoalSampleableRegion>> goals;
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-50.0, 50.0);

    for (int i = 0; i < 100; i++) {
        goals.push_back(std::make_shared<DroneEndEffectorNearTarget>(si, 0.1, Eigen::Vector3d(
                distribution(generator),
                distribution(generator),
                distribution(generator)
        )));
    }
    return goals;
}

void spawn_wall(moveit_msgs::PlanningScene &planning_scene_diff) {
    moveit_msgs::CollisionObject wallCollision;
    wallCollision.id = "wall";
    wallCollision.header.frame_id = "world";
    wallCollision.primitive_poses.resize(1);
    tf::poseEigenToMsg(Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.0)), wallCollision.primitive_poses.back());

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.1, 10.0, 200.0};
    wallCollision.primitives.push_back(primitive);
    planning_scene_diff.world.collision_objects.push_back(wallCollision);
}

std::vector<Apple> apples_around_wall() {
    std::vector<Apple> apples;

    // TODO Change this back.

    for (int i = 0; i <= 5; ++i) {
        apples.push_back({
                                 Eigen::Vector3d(0.5, (double) i, 0.0),
                                 Eigen::Vector3d(1.0, 0.0, 0.0),
                         });
    }

    for (int i = 5; i >= 0; --i) {
        apples.push_back({
                                 Eigen::Vector3d(-0.5, (double) i, 0.0),
                                 Eigen::Vector3d(-1.0, 0.0, 0.0),
                         });
    }
    return apples;
}
