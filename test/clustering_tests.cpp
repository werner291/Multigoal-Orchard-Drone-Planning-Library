
#include <gtest/gtest.h>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include "test_utils.h"
#include "../src/LeavesCollisionChecker.h"
#include "../src/multigoal/approach_table.h"
#include "../src/experiment_utils.h"
#include "../src/json_utils.h"
#include "../src/multigoal/ClusterTable.h"
#include "../src/general_utlities.h"
#include <tf2_eigen/tf2_eigen.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <random>

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

        // Load the scene data
        const Json::Value trees_data = jsonFromGzipFile("test_robots/trees/trees_2021-11-20T12:21:28Z.json.gzip");
        state_space = std::make_shared<DroneStateSpace>(spec);

        // Load some tree and branch data.
        tree_data = *treeSceneFromJson(trees_data[0]);

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
        st.setToRandomPositions();

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
    auto si = std::make_shared<ompl::base::SpaceInformation>(state_space);

    // Construct the necessary goals and take 10- samples from each.
    static const int SAMPLES_PER_GOAL = 10;
    auto goals = constructAppleGoals(si, tree_data.apples);
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
TEST_F(ClusteringTests, test_cluster_sinespacing) {

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
    auto prms = std::make_shared<ompl::geometric::PRMstar>(si);
    auto pathLengthObjective = std::make_shared<ompl::base::PathLengthOptimizationObjective>(si);
    auto sampler = std::make_shared<InformedGaussian>(state_space.get(), 2.5);
    PointToPointPlanner ptp(prms, pathLengthObjective, sampler);

    // Expand the (singleton) clusters, connecting them to others within range.
    clustering::create_cluster_candidates(ptp, samples, 3.5, clusters);

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
                    cluster.representative->get()
            );
        }
    }

    // There should be 20 density peaks in the goal samples, which should have 9 cluster members each.
    size_t count_niners = 0;
    for (const auto &cluster: clusters) {
        EXPECT_GE(9, cluster.members.size());
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

    std::unordered_set<size_t> all_goals;
    for (const auto &item: points) all_goals.insert(item.goals.begin(), item.goals.end());

    clustering::VisitationOrderSolution best_solution{
            {}, INFINITY, {}
    };

    clustering::generate_visitations<Eigen::Vector2d, PotentialGoal>(
            start_pos,
            points,
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

    std::unordered_set<size_t> goals_visited;
    for (const auto &cluster: best_solution.visit_order) {
        for (const auto &goal_id: cluster.goals_to_visit) {
            EXPECT_EQ(0, goals_visited.count(goal_id));
            goals_visited.insert(goal_id);
        }
    }
    EXPECT_EQ(all_goals, goals_visited);

}

TEST(ClusteringSubroutineTests, order_proposal_multigoal_hierarchical) {

    double start_pos = sinewaveOffset(-21);

    struct PotentialGoal {
        double v;
        std::set<size_t> goals;
    };

    struct Cluster {
        double representative;
        std::vector<PotentialGoal> points;
        std::set<size_t> reachable_goals;
    };

    std::vector<Cluster> pts;

    for (int x = -20; x <= 20; x += 5) {
        if (x % 5 == 0) pts.push_back({sinewaveOffset(x + 3), {}});
        pts.back().points.push_back({sinewaveOffset(x), {static_cast<unsigned long>(x + 100),
                                                         static_cast<unsigned long>(x * 9 / 10 + 100)}});

//        TODO:
//        Make
//        sure
//        inque
//        visitations
//        work, this
//        doesn
//        't make sense...'
    }

    std::unordered_set<size_t> all_goals;

    std::random_device r;
    std::mt19937 rng(r());
    std::shuffle(pts.begin(), pts.end(), rng);
    for (auto &item: pts) std::shuffle(item.points.begin(), item.points.end(), rng);
    for (auto &cluster: pts) {
        for (const auto &item: cluster.points) {
            cluster.reachable_goals.insert(item.goals.begin(), item.goals.end());
            all_goals.insert(item.goals.begin(), item.goals.end());
        }
    }

    clustering::VisitationOrderSolution best_solution{
            {}, INFINITY, {}
    };

    clustering::generate_visitations<double, Cluster>(
            start_pos,
            pts,
            {/*empty*/},
            [](const std::variant<double, Cluster> &a,
               const std::variant<double, Cluster> &b) {
                return abs((std::holds_alternative<double>(a) ? get<double>(a) : get<Cluster>(a).representative)
                           - (std::holds_alternative<double>(b) ? get<double>(b) : get<Cluster>(b).representative));
            },
            [](const Cluster &a) -> const std::set<size_t> & { return a.reachable_goals; },
            [&](auto soln) {
                if (soln.is_better_than(best_solution)) {
                    best_solution = soln;
                }
            });

    for (size_t i = 0; i + 1 < best_solution.visit_order.size(); ++i) {
        EXPECT_LT(pts[best_solution.visit_order[i].subcluster_index].representative,
                  pts[best_solution.visit_order[i + 1].subcluster_index].representative);
    }

    std::vector<PotentialGoal> resulting_path;

    for (size_t i = 0; i < best_solution.visit_order.size(); ++i) {

        clustering::VisitationOrderSolution best_sub_solution{
                {}, INFINITY, {}
        };

        std::optional<double> end_point{};
        if (i + 1 < best_solution.visit_order.size()) {
            *end_point = pts[best_solution.visit_order[i + 1].subcluster_index].representative;
        }

        clustering::generate_visitations<double, PotentialGoal>(
                i == 0 ? start_pos : pts[best_solution.visit_order[i - 1].subcluster_index].representative,
                pts[best_solution.visit_order[i].subcluster_index].points,
                end_point,
                [](const std::variant<double, PotentialGoal> &a,
                   const std::variant<double, PotentialGoal> &b) {
                    return abs((std::holds_alternative<double>(a) ? get<double>(a) : get<PotentialGoal>(a).v)
                               - (std::holds_alternative<double>(b) ? get<double>(b) : get<PotentialGoal>(b).v));
                },
                [](const PotentialGoal &a) -> const std::set<size_t> & { return a.goals; },
                [&](auto soln) {
                    if (soln.is_better_than(best_sub_solution)) {
                        best_sub_solution = soln;
                    }
                });

        for (const auto &item: best_sub_solution.visit_order) {
            resulting_path.push_back(pts[best_solution.visit_order[i].subcluster_index].points[item.subcluster_index]);
        }
    }

    std::unordered_set<size_t> goals_reached;

    for (size_t i = 0; i + 1 < resulting_path.size(); ++i) {
        EXPECT_LT(resulting_path[i].v, resulting_path[i + 1].v);
    }

    for (const auto &item: resulting_path) {
        for (const auto &goal_id: item.goals) {
            EXPECT_EQ(goals_reached.find(goal_id), goals_reached.end());
            goals_reached.insert(goal_id);
        }
    }

    EXPECT_EQ(all_goals, goals_reached);

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
    for (int i = -10; i <= 20; ++i) {
        apples.push_back({
                                 Eigen::Vector3d(0.5, (double) i, 5.0),
                                 Eigen::Vector3d(1.0, 0.0, 0.0),
                         });
    }
    for (int i = 20; i >= -10; --i) {
        apples.push_back({
                                 Eigen::Vector3d(-0.5, (double) i, 5.0),
                                 Eigen::Vector3d(-1.0, 0.0, 0.0),
                         });
    }

    spawnApplesInPlanningScene(0.1, apples, planning_scene_diff);
    scene->setPlanningSceneDiffMsg(planning_scene_diff);
    scene->getAllowedCollisionMatrixNonConst().setDefaultEntry("apples", true);
    scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

    auto si = initSpaceInformation(scene, drone, state_space);

    static const int SAMPLES_PER_GOAL = 10;

    auto goals = constructAppleGoals(si, apples);

    auto goal_samples = clustering::takeInitialSamples(goals, si, SAMPLES_PER_GOAL);

    // Obstacle is pretty simple, should be able to sample all goals.
    assert(goal_samples.size() >= goals.size() * SAMPLES_PER_GOAL);

    for (const auto &sample: goal_samples) {
        std::cout << sample.goal_idx << std::endl;
        EXPECT_TRUE(si->isValid(sample.state->get()));
    }

    auto prms = std::make_shared<ompl::geometric::PRMstar>(si);
    auto pathLengthObjective = std::make_shared<ompl::base::PathLengthOptimizationObjective>(si);
    auto sampler = std::make_shared<InformedGaussian>(state_space.get(), 2.5);

    PointToPointPlanner ptp(prms, pathLengthObjective, sampler);

    auto clusters = clustering::buildClusters(ptp, goal_samples);

    for (const auto &level: clusters) {
        std::unordered_set<size_t> visited_goals;
        for (const auto &cluster: level) {
            visited_goals.insert(cluster.goals_reachable.begin(), cluster.goals_reachable.end());
        }
        EXPECT_EQ(visited_goals.size(), goals.size());
    }

    ompl::base::ScopedState start_state(si);
    EXPECT_EQ(0, goal_samples[0].goal_idx);
    state_space->copyState(start_state.get(), goal_samples[0].state->get());
    start_state->as<DroneStateSpace::StateType>()->values[0] -= 1.0; // Engineer it so it's close to one of the goals, but not on it.
    start_state->as<DroneStateSpace::StateType>()->values[1] -= 1.0;

    auto ordering = clustering::visit_clusters_naive(clusters, goal_samples, start_state.get(), *si);

    std::unordered_set<size_t> visited_goals;
    for (const auto &item: ordering) {
        EXPECT_TRUE(visited_goals.find(goal_samples[item].goal_idx) == visited_goals.end());
        visited_goals.insert(goal_samples[item].goal_idx);
    }

    EXPECT_EQ(ordering.size(), goals.size());

    for (size_t i = 0; i < ordering.size(); ++i) {
        EXPECT_EQ(goal_samples[ordering[i]].goal_idx, i);
    }

}