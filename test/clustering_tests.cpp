#include <gtest/gtest.h>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <random>
#include <boost/range/combine.hpp>

#include "test_utils.h"
#include "../src/LeavesCollisionChecker.h"
#include "../src/multigoal/approach_table.h"
#include "../src/experiment_utils.h"
#include "../src/ManipulatorDroneMoveitPathLengthObjective.h"
#include "../src/DroneStateConstraintSampler.h"
#include "../src/multigoal/visitation_order_strategy.h"

/**
 * \brief This produces a set of monotonically increasing x-coordinates spaced with a smooth, wave-like pattern
 *        that consistently produces a set of dense and non-dense regions.
 */
double sinewaveOffset(int i) { return sin((double) i * M_PI / 5.0) + (double) i; }

// A simple text fixture to be used for the various tests below.
class ClusteringTests : public ::testing::Test {

protected:
    std::shared_ptr<DroneStateSpace> state_space; // Just a state space for the drone with an arm.
    TreeSceneData tree_data; // TreeSceneData for one tree.
    moveit::core::RobotModelPtr drone; // Pointer to the drone with the arm.

    void SetUp() override {
        // Get the drone model from URDF and SRDF
        drone = loadRobotModel();

        // Construct the state space for that robot.
        ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");
        state_space = std::make_shared<DroneStateSpace>(spec);

//        // Load the scene data
//        const Json::Value trees_data = jsonFromGzipFile("test_robots/trees/trees_2021-11-20T12:21:28Z.json.gzip");
//
//        // Load some tree and branch data.
//        tree_data = *treeSceneFromJson(trees_data[0]);

    }

    /**
     * Generates a random robot state. Then, it is copied 201 times, and the X-component of the base link translation
     * is set to sinewaveOffset(i), with i ranging from -100 to 100 inclusive. These are returned as a vector.
     *
     * This naturally creates states that are clustered symmetrically around samples with index == 5 (mod 10)
     * from the start of the produced vector.
     */
    std::vector<clustering::StateAtGoal> makeLineOfSampleClusters(const ompl::base::SpaceInformationPtr &si) {
        std::vector<clustering::StateAtGoal> samples;

        moveit::core::RobotState st(drone);
        DroneStateConstraintSampler::randomizeUprightWithBase(st);

        for (int i = -100; i < 100; ++i) {

            double offset = sinewaveOffset(i);

            st.setVariablePosition(0 /* X position of the base link. */, offset);

            auto state = std::make_shared<ompl::base::ScopedState<ompl::base::StateSpace> >(si);
            state_space->copyToOMPLState(state->get(), st);

            samples.push_back({0, state});
        }

        return samples;
    }

};


/// Just a simple test to make sure the sampler works at all.
TEST_F(ClusteringTests, test_sampling) {

    // We don't use any obstacles here.
    planning_scene::PlanningScenePtr scene = std::make_shared<planning_scene::PlanningScene>(drone);
    scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());
    auto si = initSpaceInformation(scene, drone, state_space);

    // Construct the necessary goals and take 10- samples from each.
    static const int SAMPLES_PER_GOAL = 10;

    std::random_device rd;
    std::mt19937 rng(rd());

    std::uniform_real_distribution<double> distr(-10.0,10.0);

    auto goals = constructAppleGoals(
            si,
            boost::copy_range<std::vector<Apple>>(boost::irange(0,10)
                | boost::adaptors::transformed([&](auto _) {
                    return Apple {
                        Eigen::Vector3d(distr(rng),distr(rng),distr(rng)),
                        Eigen::Vector3d(distr(rng),distr(rng),distr(rng)).normalized()
                    };
                })));

    auto goal_samples = clustering::takeInitialSamples(goals, si, SAMPLES_PER_GOAL);

    // Since there are no obstacles, expect to have all expected samples.
    assert(goal_samples.size() >= goals.size() * SAMPLES_PER_GOAL);

    // Check to make sure every goal is sampled from.
    std::unordered_set<size_t> goals_sampled;

    for (const auto &gs: goal_samples) {
        goals_sampled.insert(gs.goal_idx);
        // Make sure every goal is actually satisfied as advertised.
        assert(goals[gs.goal_idx]->isSatisfied(gs.state->get()));
    }

    assert(goals_sampled.size() >= goals.size()); // Make sure we're at least sampling from all goals.
}

/// Arrange the goal samples positioned using `makeLineOfSampleClusters` in order to
/// test the density-based cluster construction step.
TEST_F(ClusteringTests, DISABLED_test_cluster_sinespacing) {

    // Build a dummy empty scene with the drone in it and the type of collision detector it expects.
    auto scene = std::make_shared<planning_scene::PlanningScene>(drone);
    scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

    // Build a space information with it.
    auto si = initSpaceInformation(scene, drone, state_space);

    // Build a sine-spaced set of goals in a deterministic, predictable fashion.
    auto samples = makeLineOfSampleClusters(si);

    // Build singleton clusters out of each goal.
    auto trivial_clusters = clustering::buildTrivialClusters(samples);

    // There should be a cluster for each
    EXPECT_EQ(trivial_clusters.size(), samples.size());

    // Construct a PointToPointPlanner to be used while expanding the clusters.
    auto prms = std::make_shared<ompl::geometric::AITstar>(si);
    auto pathLengthObjective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);
    auto sampler = std::make_shared<InformedGaussian>(state_space.get(), 2.5);
    PointToPointPlanner ptp(prms, pathLengthObjective, sampler);

    clustering::NearestKPreselection preselect;
    clustering::SelectAllCandidates postselect;
    clustering::InverseDistanceFromCenterDensityStrategy density_strategy;

    // Expand the (singleton) clusters, connecting them to others within range.
    auto clusters = clustering::create_cluster_candidates(ptp, samples, trivial_clusters, preselect, postselect, 0, 0.1);

    // Assert member symmetry.
    for (size_t cluster_id = 0; cluster_id < clusters.size(); cluster_id++) {
        // Iterate over all cluster members.
        for (const auto &member: clusters[cluster_id].members) {
            auto backref = clusters[member.first].members.find(cluster_id);
            EXPECT_NE(backref, clusters[member.first].members.end());
            EXPECT_EQ(backref->second, member.second);
        }
    }

    // For the sake of test determinism, I'll be forcing the distances to simply be the distance between cluster representative states.
    for (auto &cluster: clusters) {
        for (auto &member: cluster.members) {
            member.second = state_space->distance(
                    cluster.representative->get(),
                    clusters[member.first].representative->get()
            );
        }
    }

    // There should be 20 density peaks in the goal samples, which should have 9 cluster members each.
    size_t count_niners = 0;
    for (const auto &cluster: clusters) {
        EXPECT_GE(9, cluster.members.size());
        std::cout << "Cluster size: " << cluster.members.size() << std::endl;
        if (cluster.members.size() == 9) count_niners += 1;
    }
    EXPECT_EQ(20, count_niners);

    // Compute the initial density of all clusters
    auto densities = density_strategy.computeForLayer(clusters, trivial_clusters);

    // Select representatives for the next round.
    auto selections = select_clusters(clusters, densities);

    // Make sure we have some.
    EXPECT_FALSE(selections.empty());

    // We expect the clusters to be centered on either the density peaks, or the density valleys (that form outliers)
    for (const auto &selected_idx: selections) {
        EXPECT_EQ(0, selected_idx % 5);
    }

}

TEST(ClusteringSubroutineTests, generate_combinations_test) {

    const std::vector<size_t> elements{
            1, 2, 3
    };

    const std::vector<std::vector<size_t>> expected{
            {1},
            {1, 2},
            {1, 2, 3},
            {1, 3},
            {1, 3, 2},
            {2},
            {2, 1},
            {2, 1, 3},
            {2, 3},
            {2, 3, 1},
            {3},
            {3, 2},
            {3, 2, 1},
            {3, 1},
            {3, 1, 2},
    };

    auto itr = expected.begin();

    generate_combinations<size_t, size_t>(elements, 0, [&](std::vector<size_t>::const_iterator first,
        std::vector<size_t>::const_iterator last,
                                                                       const size_t &super_value) {
        EXPECT_NE(itr, expected.end());

        auto reference_vector = (*itr++);

        EXPECT_EQ(super_value, reference_vector.size() - 1);

        EXPECT_EQ(reference_vector.size(), (last - first));

        auto ref_itr = reference_vector.begin();
        while (first != last) {
            std::cout << *first;
            EXPECT_EQ(*first, *ref_itr);
            first++;
            ref_itr++;
        }
        std::cout << std::endl;

        return reference_vector.size();
    });

    EXPECT_EQ(itr, expected.end());
}

TEST(ClusteringSubroutineTests, order_proposal_multigoal) {

    struct PotentialGoal {
        Eigen::Vector2d v;
        std::set<size_t> goals;
    };

    Eigen::Vector2d start_pos(-1.0, 0.0);

    std::vector<PotentialGoal> points{
            PotentialGoal{
                    Eigen::Vector2d(0.0, 0.0), {1, 2}
            },
            PotentialGoal{
                    Eigen::Vector2d(1.0, -0.1), {2} // Note: this goal can be skipped.
            },
            PotentialGoal{
                    Eigen::Vector2d(2.0, 0.0), {3} // Note: this goal can be skipped.
            },
            PotentialGoal{
                    Eigen::Vector2d(2.0, 1.0), {3, 4}
            },
            PotentialGoal{
                    Eigen::Vector2d(3.0, 0.0), {6}
            },
            PotentialGoal{
                    Eigen::Vector2d(4.0, 0.0), {7, 8, 9}
            },
    };

    std::set<size_t> all_goals;
    for (const auto &item: points) all_goals.insert(item.goals.begin(), item.goals.end());

    clustering::VisitationOrderSolution best_solution{
            {}, INFINITY, {}
    };

    clustering::generate_visitations<Eigen::Vector2d, PotentialGoal>(
            start_pos,
            points,
            all_goals,
            {/*empty*/},
            [](const std::variant<Eigen::Vector2d, PotentialGoal> &a,
               const std::variant<Eigen::Vector2d, PotentialGoal> &b) {
                return
                        ((std::holds_alternative<Eigen::Vector2d>(a) ? get<Eigen::Vector2d>(a) : get<PotentialGoal>(a).v)
                         - (std::holds_alternative<Eigen::Vector2d>(b) ? get<Eigen::Vector2d>(b) : get<PotentialGoal>(
                                b).v)).norm();
            },
            [](const PotentialGoal &a) -> const std::set<size_t> & { return a.goals; },
            [&](const clustering::VisitationOrderSolution &soln) {
                if (soln.is_better_than(best_solution)) {
                    best_solution = soln;
                }
            });

    const std::vector<clustering::Visitation> known_optimal {
            {0, {1, 2}},
            {3, {3, 4}},
            {4, {6}},
            {5, {7, 8, 9}}
    };

    EXPECT_EQ(known_optimal, best_solution.visit_order);

    std::set<size_t> goals_visited;
    for (const auto &cluster: best_solution.visit_order) {
        for (const auto &goal_id: cluster.goals_to_visit) {
            EXPECT_EQ(0, goals_visited.count(goal_id));
            goals_visited.insert(goal_id);
        }
    }
    EXPECT_EQ(all_goals, goals_visited);

}

TEST_F(ClusteringTests, distance_matrix_test) {

    // Build a dummy empty scene with the drone in it and the type of collision detector it expects.
    auto scene = std::make_shared<planning_scene::PlanningScene>(drone);
    scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

    // Build a space information with it.
    auto si = initSpaceInformation(scene, drone, state_space);

    std::vector<clustering::Cluster> context_cluster;

    moveit::core::RobotState st(drone);
    DroneStateConstraintSampler::randomizeUprightWithBase(st);

    clustering::Cluster collective {
        std::make_shared<ompl::base::ScopedState<>>(si),
        {},
        {}
    };

    size_t GRID_SIZE = 2;

    for (int x = 0; x < GRID_SIZE; ++x) {
        for (int y = 0; y < GRID_SIZE; ++y) {
            st.setVariablePosition(0,(double)x);
            st.setVariablePosition(1,(double)y);

            auto repr = std::make_shared<ompl::base::ScopedState<>>(si);

            state_space->copyToOMPLState(repr->get(), st);

            context_cluster.push_back({repr, {}, {}});

            collective.members[x*GRID_SIZE+y] = 1.0;
        }
    }

    auto prms = std::make_shared<ompl::geometric::AITstar>(si);
    auto pathLengthObjective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);
    auto sampler = std::make_shared<InformedGaussian>(state_space.get(), 2.5);
    PointToPointPlanner ptp(prms, pathLengthObjective, sampler);

    auto dm = clustering::computeDistanceMatrix(ptp, collective, context_cluster, 0.2);

    ASSERT_EQ(std::pow(GRID_SIZE,4),dm.size());

    ompl::RNG rng;

    for (size_t i : boost::irange(0,100)) {
        size_t x1 = rng.uniformInt(0,GRID_SIZE-1);
        size_t x2 = rng.uniformInt(0,GRID_SIZE-1);
        size_t y1 = rng.uniformInt(0,GRID_SIZE-1);
        size_t y2 = rng.uniformInt(0,GRID_SIZE-1);

        std::cout << x1 << "," << y1 << std::endl;
        std::cout << "2: " << x2 << "," << y2 << std::endl;

        double expected_distance =
            std::sqrt((std::pow((double)x1-(double)x2,2)
                     + std::pow((double)y1-(double)y2,2)));

        EXPECT_NEAR(expected_distance,dm[std::make_pair(x1*GRID_SIZE+y1,x2*GRID_SIZE+y2)],0.1);
        EXPECT_NEAR(expected_distance,dm[std::make_pair(x2*GRID_SIZE+y2,x1*GRID_SIZE+y1)],0.1);
    }
}

void check_cluster_members_in_dm(const clustering::Cluster& cluster, const clustering::DistanceMatrix& dm) {
    // For every pair of cluster members, the distance matrix should have an entry.
    for (const auto &a : cluster.members) {
        for (const auto &b : cluster.members) {
            EXPECT_NE(0,dm.count(std::make_pair(a.first, b.first)));
        }
    }
}

void check_cluster_distance_matrix_hierarchy(const clustering::ClusterHierarchy& clusters,
                                             const clustering::DistanceMatrixHierarchy& distanceMatrices) {
    // Check that all goals are reachable at every level. (Should be possible with the wall test)
    for (size_t level_id = 0; level_id + 1 < clusters.size(); level_id++) {

        // Check to make sure that the distance matrices provide a distance for every pair of cluster members.
        for (auto pair : boost::combine(clusters[level_id], distanceMatrices[level_id])) {
            check_cluster_members_in_dm(pair.get<0>(), pair.get<1>());
        }
    }
}

std::unordered_set<size_t> visited_goals_in_clusters(const std::vector<clustering::Cluster>& clusters) {
    std::unordered_set<size_t> visited_goals;
    for (auto cl:clusters) {
        visited_goals.insert(cl.goals_reachable.begin(), cl.goals_reachable.end());
    }
    return visited_goals;
}

/**
 * This function checks whether the provided clusters are non-overlapping.
 * Note that this may not always be the case, depending on the clustering method...
 */
void check_clusters_exclusive(const std::vector<clustering::Cluster>& clusters) {

    std::unordered_set<size_t> found_member_ids;

    for (const auto& cluster: clusters) {
        for (const auto& [mem_id, dist]: cluster.members) {
            EXPECT_EQ(0,found_member_ids.count(mem_id));
            found_member_ids.insert(mem_id);
        }
    }

}

/// In this test, the planning scene consists of a single, tall and thin wall.
/// Apples are arranged in a line on both sides; the line extends to the end of the wall on one side.
///
/// The optimal visitation order is pretty obvious: just go down one line, around the wall, then follow the line
/// on the other side. The algorithm must discover this pathway.
///
/// Note: This case is highly adversarial to the Euclidean nearest-neighbours algorithm since it would try
/// to traverse the wall many times.
TEST_F(ClusteringTests, test_full_wall) {

//#define PUBLISH_RVIZ

    auto[scene,apples] = createWallApplePlanningScene(drone);

//#ifdef PUBLISH_RVIZ
//
//    int zero = 0;
//    ros::init(zero, nullptr, "full_wall_test");
//    ros::AsyncSpinner spinner(1);
//    spinner.start();
//    ros::NodeHandle node_handle;
//
//    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
//    while (planning_scene_diff_publisher.getNumSubscribers() == 0) {
//        ros::Duration(0.5).sleep();
//    }
//    moveit_msgs::PlanningScene scene_msg;
//    scene->getPlanningSceneMsg(scene_msg);
//    planning_scene_diff_publisher.publish(scene_msg);
//#endif

    auto si = initSpaceInformation(scene, drone, state_space);

    static const int SAMPLES_PER_GOAL = 20;

    auto goals = constructAppleGoals(si, apples);

    auto goal_samples = clustering::takeInitialSamples(goals, si, SAMPLES_PER_GOAL);

    // Obstacle is pretty simple, should be able to sample all goals.
    assert(goal_samples.size() >= goals.size() * SAMPLES_PER_GOAL);

    for (const auto &sample: goal_samples) {
        EXPECT_TRUE(si->isValid(sample.state->get()));
    }

    auto prms = std::make_shared<ompl::geometric::AITstar>(si);
    auto pathLengthObjective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);
    auto sampler = std::make_shared<InformedGaussian>(state_space.get(), 2.5);

    PointToPointPlanner ptp(prms, pathLengthObjective, sampler);

    clustering::SearchRTryN preselection({0.1,1.5}, 10);
    clustering::SelectByExponentialRadius postselect_exprad({0.1, 1.5});
    clustering::InverseDistanceFromCenterDensityStrategy density_strategy;

    auto clusters = clustering::buildClusters(ptp, goal_samples, preselection, postselect_exprad, density_strategy);

    std::cout << "Clusters built." << std::endl;

    auto distanceMatrices = computeAllDistances(ptp, clusters);

    std::cout << "Distance matrices computed." << std::endl;

    dump_clusters(clusters, state_space);

    for (size_t level_id = 0; level_id + 1 < clusters.size(); level_id++) {
        EXPECT_EQ(visited_goals_in_clusters(clusters[level_id]).size(), goals.size());
    }

    check_cluster_distance_matrix_hierarchy(clusters, distanceMatrices);

    // On every layer, ensure that every item on the previous layer is in at least one cluster.
    for (size_t level_id = clusters.size(); level_id + 1 < clusters.size(); level_id++) {
        std::unordered_set<size_t> previous_layer_members_found;

        for (const auto& cl: clusters[level_id]) {
            for (const auto& [mem_id, dist]: cl.members) {
                previous_layer_members_found.insert(mem_id);
            }
        }

        ASSERT_EQ(previous_layer_members_found.size(), clusters[level_id+1].size());

        for (size_t i = 0; i < clusters[level_id+1].size(); ++i) {
            EXPECT_NE(previous_layer_members_found.find(i),
                      previous_layer_members_found.end());
        }
    }

    auto start_state = std::make_shared<ompl::base::ScopedState<ompl::base::StateSpace> >(si);
    EXPECT_EQ(0, goal_samples[0].goal_idx);
    state_space.get()->copyState(start_state->get(), goal_samples[0].state->get());
    start_state->get()->as<DroneStateSpace::StateType>()->values[0] -= 1.0; // Engineer it so it's close to one of the goals, but not on it.
    start_state->get()->as<DroneStateSpace::StateType>()->values[1] -= 1.0;

    auto ordering_layers = clustering::determine_visitation_order(
            start_state,clusters,
            clustering::InClusterPrecomputedDistanceMetricWithFallback(clusters, distanceMatrices));

    std::ofstream order_file("analysis/ordering.txt");
    for (const auto& layer: ordering_layers) {
        for (const auto& item: layer) {
            order_file << item << ",";
        }
        order_file << std::endl;
    }
    order_file.close();

    auto ordering = ordering_layers.back();

    std::unordered_set<size_t> visited_goals;
    for (const auto &item: ordering) {
        EXPECT_TRUE(visited_goals.find(goal_samples[item].goal_idx) == visited_goals.end());
        visited_goals.insert(goal_samples[item].goal_idx);
    }

    EXPECT_EQ(ordering.size(), goals.size());

    for (unsigned long i : ordering) {
        moveit::core::RobotState st(drone);
        state_space->copyToRobotState(st, goal_samples[i].state->get());

        std::cout << goal_samples[i].goal_idx << " - "
                  << st.getVariablePosition(0) << ", "
                  << st.getVariablePosition(1) << ", "
                  << st.getVariablePosition(2) << std::endl;
    }

    for (size_t i = 0; i < ordering.size(); ++i) {
        EXPECT_EQ(goal_samples[ordering[i]].goal_idx, i);
    }

//    #ifdef PUBLISH_RVIZ
//    ros::Publisher cluster_marker_publisher = node_handle.advertise<visualization_msgs::MarkerArray>("/clusters_markers", 1);
//    while (cluster_marker_publisher.getNumSubscribers() == 0) {
//        ros::Duration(0.5).sleep();
//    }
//    size_t layer_i = 0;
//
//    while (true) {
//
//        layer_i = (layer_i + 1 ) % clusters.size();
//
//        visualization_msgs::MarkerArray ma;
//        for (const auto& cluster : clusters[layer_i]) {
//            moveit::core::RobotState st(drone);
//            state_space->copyToRobotState(st, cluster.representative->get());
//
//            visualization_msgs::MarkerArray state_markers = markers_for_state(st);
//            for (auto marker : state_markers.markers) {
//                marker.id = ma.markers.size();
//                ma.markers.push_back(marker);
//            }
//        }
//
//        cluster_marker_publisher.publish(ma);
//
//        ros::Duration(0.5).sleep();
//
//        {
//            visualization_msgs::MarkerArray delete_all;
//            visualization_msgs::Marker delete_all_marker;
//            delete_all_marker.action = visualization_msgs::Marker::DELETEALL;
//            delete_all.markers.push_back(delete_all_marker);
//            cluster_marker_publisher.publish(delete_all);
//        }
//    }
//    #endif
}
