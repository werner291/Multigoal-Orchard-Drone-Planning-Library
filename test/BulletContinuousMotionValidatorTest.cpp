
#include <gtest/gtest.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <boost/graph/astar_search.hpp>

#include "../src/BulletContinuousMotionValidator.h"
#include "test_utils.h"
#include "../src/LeavesCollisionChecker.h"
#include "../src/multigoal/approach_table.h"
#include "../src/utilities/experiment_utils.h"
#include "../src/MoveitPathLengthObjective.h"


TEST(LeavesCollisionCheckerTest, test_collisions) {

    auto robot = loadRobotModel();

    LeavesCollisionChecker lcc({

                                       Eigen::Vector3d(-1.0, 0.0, -1.0),
                                       Eigen::Vector3d(1.0, 0.0, -1.0),
                                       Eigen::Vector3d(-1.0, 0.0, 1.0),

                                       Eigen::Vector3d(-1.0, 1.0, -1.0),
                                       Eigen::Vector3d(1.0, 1.0, -1.0),
                                       Eigen::Vector3d(-1.0, 1.0, 1.0),

                                       Eigen::Vector3d(-1.0, 5.0, -1.0),
                                       Eigen::Vector3d(1.0, 5.0, -1.0),
                                       Eigen::Vector3d(-1.0, 5.0, 1.0),

                               });

    moveit::core::RobotState st(robot);
    st.setVariablePositions({
                                    0.0, -2.0, 0.0, // Base link translation
                                    0.0, 0.0, 0.0, 1.0, // Base link orientation
                                    0.0, 0.0, 0.0, 0.0 // Arm joint angles
                            });
    st.update(true);

    ASSERT_TRUE(lcc.checkLeafCollisions(st).empty());

    st.setVariablePositions({
                                    0.0, 0.0, 0.0, // Base link translation
                                    0.0, 0.0, 0.0, 1.0, // Base link orientation
                                    0.0, 0.0, 0.0, 0.0 // Arm joint angles
                            });
    st.update(true);
    auto collisions_1 = lcc.checkLeafCollisions(st);
    ASSERT_EQ(1, collisions_1.size());

    st.setVariablePositions({
                                    0.001, 0.05, 0.0, // Base link translation
                                    0.0, 0.0, 0.0, 1.0, // Base link orientation
                                    0.0, 0.0, 0.0, 0.0 // Arm joint angles
                            });
    st.update(true);
    auto collisions_2 = lcc.checkLeafCollisions(st);
    ASSERT_EQ(collisions_1, collisions_2);

    // TODO: Need to figure out what the IDs actually mean. I can observe they're equal, so that's good at least.

}

TEST(AStarRoadmapGoals, test_astar) {

    using namespace boost;

    struct Vertex {
        Eigen::Vector2d pos;
        size_t goal;
    };

    const size_t NOT_GOAL = SIZE_MAX;

    ompl::RNG rng;

    typedef adjacency_list<listS, vecS, undirectedS, Vertex, property<edge_weight_t, double> > Graph;
    Graph gr;
    typedef Graph::vertex_descriptor VD;

    struct GNATNode {
        VD vd;
        Eigen::Vector2d pos;

        bool operator==(const GNATNode &other) const {
            return vd == other.vd && pos == other.pos;
        }

        bool operator!=(const GNATNode &other) const {
            return !(*this == other);
        }
    };

    // Place all apples into a Geometric Nearest-Neighbour access tree, using Euclidean distance.
    ompl::NearestNeighborsGNAT<GNATNode> unvisited_nn;
    unvisited_nn.setDistanceFunction([](const GNATNode &a, const GNATNode &b) {
        return (a.pos - b.pos).norm();
    });

    VD start;

    for (int i = 0; i < 10000; i++) {
        Eigen::Vector2d v_pos(rng.uniformReal(-100.0, 100.0), rng.uniformReal(100.0, 100.0));

        auto vd = boost::add_vertex(Vertex{
                v_pos,
                NOT_GOAL
        }, gr);

        if (i == 0) {
            start = vd;
        }

        unvisited_nn.add({
                                 vd,
                                 v_pos
                         });

        std::vector<GNATNode> nearestK;
        unvisited_nn.nearestK(GNATNode{vd, v_pos}, 5, nearestK);
        for (const auto &near: nearestK) {
            boost::add_edge(vd, near.vd, (v_pos - near.pos).norm(), gr);
        }

    }

    std::vector<Eigen::Vector2d> goal_centers(10);
    for (size_t i = 0; i < goal_centers.size(); ++i) {
        auto c = goal_centers[i];
        c.x() = rng.uniformReal(-100.0, 100.0);
        c.y() = rng.uniformReal(-100.0, 100.0);

        auto vd = boost::add_vertex(Vertex{c, i}, gr);

        std::vector<GNATNode> nearestK;
        unvisited_nn.nearestK(GNATNode{vd, c}, 5, nearestK);
        for (const auto &near: nearestK) {
            boost::add_edge(vd, near.vd, (c - near.pos).norm(), gr);
        }
    }

    assert(goal_centers.size() >= 2);

    std::vector<double> goal_distances(goal_centers.size() - 1);
    for (size_t i = 0; i < goal_distances.size(); ++i) {
        goal_distances[i] = (goal_centers[i] - goal_centers[i + 1]).norm();
    }

//    std::priority_queue<V

}

