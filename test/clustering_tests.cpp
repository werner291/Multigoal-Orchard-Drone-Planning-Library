
#include <gtest/gtest.h>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include "test_utils.h"
#include "../src/LeavesCollisionChecker.h"
#include "../src/multigoal/approach_table.h"
#include "../src/experiment_utils.h"
#include "../src/json_utils.h"
#include "../src/multigoal/ClusterTable.h"
#include <tf2_eigen/tf2_eigen.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

class ClusteringTests : public ::testing::Test {

protected:
    std::shared_ptr<DroneStateSpace> state_space;
    TreeSceneData tree_data;
    moveit::core::RobotModelPtr drone;
    GoalSet goals;
//    ompl::base::SpaceInformationPtr si;

    void SetUp() override {
        // Get the drone model from URDF and SRDF
        drone = loadRobotModel();

        ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");

        // Load the scene data
        const Json::Value trees_data = jsonFromGzipFile("test_robots/trees/trees_2021-11-20T12:21:28Z.json.gzip");
        state_space = std::make_shared<DroneStateSpace>(spec);

        tree_data = *treeSceneFromJson(trees_data[0]);

        auto planning_scene = constructPlanningScene(tree_data, drone);
//            si = initSpaceInformation(planning_scene, drone, state_space);

//            si =

//            goals = constructAppleGoals(si,tree_data.apples);
    }

    std::vector<clustering::StateAtGoal> makeLineOfSampleClusters(const ompl::base::SpaceInformationPtr &si) {
        std::vector<clustering::StateAtGoal> samples;

        moveit::core::RobotState st(drone);
        st.setToRandomPositions();

        for (int i = -100; i < 100; ++i) {

            //            std::cout << "Offset: " << offset << std::endl;

            st.setVariablePosition(0 /* X position of the base link. */, sinewaveOffset(i));

            auto state = std::make_shared<ompl::base::ScopedState<ompl::base::StateSpace> >(si);
            state_space->copyToOMPLState(state->get(), st);

            samples.push_back({0, state});
        }

        return samples;
    }

    /**
     * \brief This produces a set of monotonically increasing x-coordinates spaced with a smooth, wave-like pattern
     *        that consistently produces a set of dense and non-dense regions.
     */
    static double sinewaveOffset(int i) { return sin((double) i * M_PI / 5.0) + (double) i; }

};

TEST_F(ClusteringTests, test_sampling) {
    auto si = std::make_shared<ompl::base::SpaceInformation>(state_space);

    static const int SAMPLES_PER_GOAL = 10;

    auto goal_samples = clustering::takeInitialSamples(goals, si, SAMPLES_PER_GOAL);

    // Since there are no obstacles, expect to have all expected samples.
    assert(goal_samples.size() >= goals.size() * SAMPLES_PER_GOAL);

    std::unordered_set<size_t> goals_sampled;

    for (const auto &gs: goal_samples) {
        goals_sampled.insert(gs.goal_idx);
        assert(goals[gs.goal_idx]->isSatisfied(gs.state->get()));
    }

    assert(goals_sampled.size() >= goals.size()); // Make sure we're at least sampling from all goals.
}

TEST_F(ClusteringTests, test_cluster_sinespacing) {
    auto scene = std::make_shared<planning_scene::PlanningScene>(drone);
    scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());
    auto si = initSpaceInformation(scene, drone, state_space);

    auto samples = makeLineOfSampleClusters(si);

    auto clusters = clustering::buildTrivialClusters(samples);

    ASSERT_EQ(clusters.size(), samples.size());

    auto prms = std::make_shared<ompl::geometric::PRMstar>(si);
    auto pathLengthObjective = std::make_shared<ompl::base::PathLengthOptimizationObjective>(si);
    auto sampler = std::make_shared<InformedGaussian>(state_space.get(), 2.5);

    PointToPointPlanner ptp(prms, pathLengthObjective, sampler);

    clustering::expandClusters(ptp, samples, 1.1, clusters);

    size_t count_fivers = 0;
    size_t count_singletons = 0;
    for (const auto &cluster: clusters) {
        std::cout << "Cluster size:" << cluster.members.size() << std::endl;
        ASSERT_GE(5, cluster.members.size());
        if (cluster.members.size() == 5) count_fivers += 1;
        if (cluster.members.size() == 1) count_singletons += 1;
    }
    ASSERT_EQ(20, count_fivers);

    auto densities = computeDensities(clusters);

    std::cout << "Densities: ";

    for (const auto &item: densities)
        std::cout << item << ", ";

    std::cout << std::endl;

    auto maxima = findDensityMaxima(clusters, densities);

    ASSERT_EQ(maxima.size(), count_fivers + count_singletons);

}

TEST_F(ClusteringTests, test_full_wall) {
    planning_scene::PlanningScenePtr scene = std::make_shared<planning_scene::PlanningScene>(drone);
    scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

    moveit_msgs::PlanningScene planning_scene_diff;

    moveit_msgs::CollisionObject wallCollision;
    wallCollision.id = "floor";
    wallCollision.header.frame_id = "world";
    wallCollision.primitive_poses.push_back(Eigen::toMsg(Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 5.0))));

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.1, 20.0, 10.0};
    wallCollision.primitives.push_back(primitive);
    planning_scene_diff.world.collision_objects.push_back(wallCollision);

    std::vector<Apple> apples;
    for (size_t i = -10; i <= 20; ++i) {
        apples.push_back({
                                 Eigen::Vector3d(0.2, (double) i, 5.0),
                                 Eigen::Vector3d(1.0, 0.0, 0.0),
                         });
        apples.push_back({
                                 Eigen::Vector3d(-0.2, (double) i, 5.0),
                                 Eigen::Vector3d(-1.0, 0.0, 0.0),
                         });
    }

    spawnApplesInPlanningScene(0.1, apples, planning_scene_diff);
    scene->setPlanningSceneMsg(planning_scene_diff);
    scene->getAllowedCollisionMatrixNonConst().setDefaultEntry("apples", true);
    scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

//    Cluster

}
