#include <eigen_conversions/eigen_msg.h>
#include <fstream>
#include <gtest/gtest.h>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <random>

#include "../src/DroneStateConstraintSampler.h"
#include "../src/LeavesCollisionChecker.h"
#include "../src/utilities/experiment_utils.h"
#include "../src/multigoal/ClusterTable.h"
#include "../src/multigoal/approach_table.h"
#include "test_utils.h"

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

    for (int i = 0; i <= 5; i += 1) {
        apples.push_back({
                                 Eigen::Vector3d(0.5, (double) i, 0.0),
                                 Eigen::Vector3d(1.0, 0.0, 0.0),
                         });
    }

    for (int i = 5; i >= 0; i -= 1) {
        apples.push_back({
                                 Eigen::Vector3d(-0.5, (double) i, 0.0),
                                 Eigen::Vector3d(-1.0, 0.0, 0.0),
                         });
    }
    return apples;
}

void dump_clusters(const std::vector<std::vector<clustering::Cluster>> clusters,
                   const std::shared_ptr<DroneStateSpace> state_space) {

    std::ofstream fout;
    fout.open("analysis/cluster_pts.txt");
    assert(fout.is_open());

    for (auto level : clusters) {
        std::unordered_set<size_t> visited_goals;

        fout << "========= Level =========" << std::endl;

        for (auto & cluster : level) {
            visited_goals.insert(cluster.goals_reachable.begin(), cluster.goals_reachable.end());

            moveit::core::RobotState st(state_space->getRobotModel());
            state_space->copyToRobotState(st, cluster.representative->get());

            st.update(true);

            fout << st.getVariablePosition(0) << ", "
                << st.getVariablePosition(1) << ", "
                << st.getVariablePosition(2) << std::endl;

            for (const auto &item : cluster.members) {
                fout << item.first << ",";
            }
            fout << std::endl;

            for (const auto &item : cluster.goals_reachable) {
                fout << item << ",";
            }
            fout << std::endl;
        }
    }
    fout.close();
}

WallAppleScene createWallApplePlanningScene(const moveit::core::RobotModelConstPtr& drone) {

    auto scene = std::make_shared<planning_scene::PlanningScene>(drone);
    scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

    auto apples = apples_around_wall();
    moveit_msgs::PlanningScene planning_scene_diff;
    spawn_wall(planning_scene_diff);
    spawnApplesInPlanningScene(0.1, apples, planning_scene_diff);
    scene->setPlanningSceneDiffMsg(planning_scene_diff);
    scene->getAllowedCollisionMatrixNonConst().setDefaultEntry("apples", true);

    return {scene,apples};

}