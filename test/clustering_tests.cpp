#include <gtest/gtest.h>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <random>

#include "test_utils.h"
#include "../src/LeavesCollisionChecker.h"
#include "../src/multigoal/approach_table.h"
#include "../src/experiment_utils.h"
#include "../src/multigoal/ClusterTable.h"
#include "../src/ManipulatorDroneMoveitPathLengthObjective.h"
#include "../src/DroneStateConstraintSampler.h"

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
    auto clusters = clustering::buildTrivialClusters(samples);

    // There should be a cluster for each
    EXPECT_EQ(clusters.size(), samples.size());

    // Construct a PointToPointPlanner to be used while expanding the clusters.
    auto prms = std::make_shared<ompl::geometric::AITstar>(si);
    auto pathLengthObjective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);
    auto sampler = std::make_shared<InformedGaussian>(state_space.get(), 2.5);
    PointToPointPlanner ptp(prms, pathLengthObjective, sampler);

    // Expand the (singleton) clusters, connecting them to others within range.
   clusters = clustering::create_cluster_candidates(ptp, samples, 3.5, clusters, 11, 0.1);

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
    auto densities = computeDensities(clusters);

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

    clustering::generate_combinations<size_t, size_t>(elements, 0, [&](std::vector<size_t>::const_iterator first,
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

    const std::vector<clustering::Visitation> known_optimal{
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

TEST(ClusteringSubroutineTests, order_proposal_multigoal_hierarchical) {

    double start_pos = -201.0;

    struct PotentialGoal { double v; size_t goal_id; };
    typedef clustering::GenericCluster<double> Cluster;

    std::set<size_t> all_goals;

    std::vector<PotentialGoal> points;
    for (int x = -200; x <= 200; x += 1) {

        size_t goal_i = static_cast<unsigned long>(x /* * 9 / 10 */ +  1000);

        points.push_back({(double) x, goal_i});

        all_goals.insert(goal_i);
    }

    std::vector<std::vector<Cluster>> hierarchy;
    {
        hierarchy.emplace_back(/*empty*/);

        for (size_t idx = 0; idx < points.size(); ++idx) {
            hierarchy.back().push_back({
                                               points[idx].v, {{idx, 1.0}}, {points[idx].goal_id}
                                       });
        }

        while (hierarchy.back().size() > 1) {
            std::vector<Cluster> new_layer;

            for (size_t idx = 0; idx < hierarchy.back().size(); idx += 5) {

                size_t range_end = std::min(idx + 5, hierarchy.back().size());
                size_t middle = idx + (range_end - idx) / 2;

                Cluster new_cluster;

                new_cluster.representative = hierarchy.back()[middle].representative;

                for (size_t inner_idx = idx; inner_idx < range_end; ++inner_idx) {
                    new_cluster.members[inner_idx] = 1.0;
                    new_cluster.goals_reachable.insert(hierarchy.back()[inner_idx].goals_reachable.begin(),
                                                       hierarchy.back()[inner_idx].goals_reachable.end());
                }

                new_layer.push_back(std::move(new_cluster));
            }

            std::set<size_t> reachable_in_layer;
            for (const auto &item : new_layer) {
                reachable_in_layer.insert(item.goals_reachable.begin(), item.goals_reachable.end());
            }
            EXPECT_EQ(all_goals, reachable_in_layer);

            hierarchy.push_back(std::move(new_layer));

        }

        std::reverse(hierarchy.begin(), hierarchy.end());
    }

    auto traversal_order = clustering::determine_visitation_order<double>(
            start_pos,
            hierarchy,
            [&](const auto& a, const auto& b, size_t t, size_t cluster_i) {

                const auto layer = hierarchy[t];

                const auto repr_a = std::holds_alternative<double>(a) ? std::get<double>(a) : layer[std::get<size_t>(
                        a)].representative;

                const auto repr_b = std::holds_alternative<double>(b) ? std::get<double>(b) : layer[std::get<size_t>(
                        b)].representative;

                return std::abs(repr_a-repr_b);
            }
    );

    auto resulting_path = boost::copy_range<std::vector<PotentialGoal>>(
            traversal_order | boost::adaptors::transformed([&](size_t i) { return points[i]; })
            );
//
//    EXPECT_EQ(points.size(), hierarchy.back().size());
//
//    for (const auto& point_idx : previous_layer_order) {
//
//        std::cout <<  "l: " << point_idx.subcluster_index
//        << " s: " << point_idx.goals_to_visit.size()
//        << " v: " << points[point_idx.subcluster_index].v << std::endl;
//
//        EXPECT_EQ(1, point_idx.goals_to_visit.size());
//
//        resulting_path.push_back(points[point_idx.subcluster_index]);
//    }

    std::set<size_t> goals_reached;

    for (size_t i = 0; i + 1 < resulting_path.size(); ++i) {
        EXPECT_LT(resulting_path[i].v, resulting_path[i + 1].v);
    }

    for (const auto &item: resulting_path) {
        std::cout << "Goal reached: " << item.goal_id << std::endl;
        EXPECT_EQ(goals_reached.find(item.goal_id), goals_reached.end());
        goals_reached.insert(item.goal_id);
    }

    EXPECT_EQ(all_goals, goals_reached);

    for (auto goal_i : all_goals) {
        if (goals_reached.count(goal_i) == 0)
            std::cout << "Goal missed: " << goal_i << std::endl;
    }

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

/// In this test, the planning scene consists of a single, tall and thin wall.
/// Apples are arranged in a line on both sides; the line extends to the end of the wall on one side.
///
/// The optimal visitation order is pretty obvious: just go down one line, around the wall, then follow the line
/// on the other side. The algorithm must discover this pathway.
///
/// Note: This case is highly adversarial to the Euclidean nearest-neighbours algorithm since it would try
/// to traverse the wall many times.
TEST_F(ClusteringTests, test_full_wall) {

    std::unordered_set<size_t> s;
    EXPECT_EQ(s.find(5), s.end());

    planning_scene::PlanningScenePtr scene = std::make_shared<planning_scene::PlanningScene>(drone);
    scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

    moveit_msgs::PlanningScene planning_scene_diff;

    spawn_wall(planning_scene_diff);

    auto apples = apples_around_wall();
    spawnApplesInPlanningScene(0.1, apples, planning_scene_diff);

    scene->setPlanningSceneDiffMsg(planning_scene_diff);
    scene->getAllowedCollisionMatrixNonConst().setDefaultEntry("apples", true);
    scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

    auto si = initSpaceInformation(scene, drone, state_space);

    static const int SAMPLES_PER_GOAL = 5;

    auto goals = constructAppleGoals(si, apples);

    auto goal_samples = clustering::takeInitialSamples(goals, si, SAMPLES_PER_GOAL);

    // Obstacle is pretty simple, should be able to sample all goals.
    assert(goal_samples.size() >= goals.size() * SAMPLES_PER_GOAL);

    for (const auto &sample: goal_samples) {
        std::cout << sample.goal_idx << std::endl;
        EXPECT_TRUE(si->isValid(sample.state->get()));
    }

    auto prms = std::make_shared<ompl::geometric::AITstar>(si);
    auto pathLengthObjective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);
    auto sampler = std::make_shared<InformedGaussian>(state_space.get(), 2.5);

    PointToPointPlanner ptp(prms, pathLengthObjective, sampler);

    auto clusters = clustering::buildClusters(ptp, goal_samples);

    std::cout << "Clusters built." << std::endl;

    auto distanceMatrices = clustering::computeAllDistances(ptp, clusters);

    std::cout << "Distance matrices computed." << std::endl;

    std::ofstream fout;
    fout.open("../analysis/cluster_pts.txt");
    assert(fout.is_open());

    for (size_t level_id = 0; level_id < clusters.size(); level_id++) {
        auto level = clusters[level_id];
        std::unordered_set<size_t> visited_goals;

        fout << "========= Level =========" << std::endl;

        for (size_t cluster_id = 0; cluster_id < level.size(); cluster_id++) {
            const auto& cluster = level[cluster_id];
            visited_goals.insert(cluster.goals_reachable.begin(), cluster.goals_reachable.end());

            moveit::core::RobotState st(drone);
            state_space->copyToRobotState(st, cluster.representative->get());

            st.update(true);

            fout << st.getVariablePosition(0) << ", "
                 << st.getVariablePosition(1) << ", "
                 << st.getVariablePosition(2) << std::endl;

            for (const auto &item : cluster.members) {
                fout << item.first << ",";
            }
            fout << std::endl;

            if (level_id + 1 < clusters.size()) {
                for (const auto &a : cluster.members) {
                    for (const auto &b : cluster.members) {
                        EXPECT_NE(
                            distanceMatrices[level_id][cluster_id].find(std::make_pair(a.first, b.first)),
                            distanceMatrices[level_id][cluster_id].end()
                        );
                    }
                }
            }

            EXPECT_LE(cluster.members.size(), clustering::DEFAULT_CLUSTER_SIZE);


        }

        EXPECT_EQ(visited_goals.size(), goals.size());
    }
    fout.close();

    auto start_state = std::make_shared<ompl::base::ScopedState<ompl::base::StateSpace> >(si);
    EXPECT_EQ(0, goal_samples[0].goal_idx);
    state_space.get()->copyState(start_state->get(), goal_samples[0].state->get());
    start_state->get()->as<DroneStateSpace::StateType>()->values[0] -= 1.0; // Engineer it so it's close to one of the goals, but not on it.
    start_state->get()->as<DroneStateSpace::StateType>()->values[1] -= 1.0;

    //                // Distance function. Either looks up the representative of a cluster based on index,
    //                // or uses the given reference point directly.
    //                auto distance = [&](const std::variant<P, size_t> &a,
    //                                    const std::variant<P, size_t> &b) {
    //                    const auto repr_a = std::holds_alternative<P>(a) ? std::get<P>(a) : layer[std::get<size_t>(
    //                            a)].representative;
    //                    const auto repr_b = std::holds_alternative<P>(b) ? std::get<P>(b) : layer[std::get<size_t>(
    //                            b)].representative;
    //                    return distanceFn(repr_a, repr_b);
    //                };

    auto distance_straight = [&](const auto& a, const auto& b, size_t layer_i, size_t _in_cluster_i) {
        const auto layer = clusters[layer_i];

        const auto repr_a = std::holds_alternative<ompl::base::ScopedStatePtr>(a) ? std::get<ompl::base::ScopedStatePtr>(a) : layer[std::get<size_t>(
                a)].representative;

        const auto repr_b = std::holds_alternative<ompl::base::ScopedStatePtr>(b) ? std::get<ompl::base::ScopedStatePtr>(b) : layer[std::get<size_t>(
                b)].representative;

        return repr_a->distance(repr_b->get());
    };

    auto distance = [&](const auto& a, const auto& b, size_t layer_i, size_t in_cluster_i) {

        if (std::holds_alternative<ompl::base::ScopedStatePtr>(a) || std::holds_alternative<ompl::base::ScopedStatePtr>(b)) {
            return distance_straight(a, b, layer_i, in_cluster_i);
        } else {
            auto key = std::make_pair(std::get<size_t>(a),std::get<size_t>(b));

            EXPECT_NE(clusters[layer_i-1][in_cluster_i].members.find(key.first),
                      clusters[layer_i-1][in_cluster_i].members.end());

            EXPECT_NE(clusters[layer_i-1][in_cluster_i].members.find(key.second),
                      clusters[layer_i-1][in_cluster_i].members.end());

            EXPECT_NE(distanceMatrices[layer_i-1][in_cluster_i].find(key),distanceMatrices[layer_i][in_cluster_i].end());

            return distanceMatrices[layer_i-1][in_cluster_i][key];
        }
    };

    auto ordering = clustering::determine_visitation_order<ompl::base::ScopedStatePtr>(start_state,clusters,distance);

    std::unordered_set<size_t> visited_goals;
    for (const auto &item: ordering) {
        EXPECT_TRUE(visited_goals.find(goal_samples[item].goal_idx) == visited_goals.end());
        visited_goals.insert(goal_samples[item].goal_idx);
    }

    EXPECT_EQ(ordering.size(), goals.size());

    for (size_t i = 0; i < ordering.size(); ++i) {
        moveit::core::RobotState st(drone);
        state_space->copyToRobotState(st, goal_samples[ordering[i]].state->get());
        std::cout << st.getVariablePosition(0) << ", "
                  << st.getVariablePosition(1) << ", "
                  << st.getVariablePosition(2) << std::endl;
    }

    for (size_t i = 0; i < ordering.size(); ++i) {
        EXPECT_EQ(goal_samples[ordering[i]].goal_idx, i);
    }
}
