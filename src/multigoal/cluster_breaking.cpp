//
// Created by werner on 30-09-21.
//

#include "cluster_breaking.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

typedef ompl::base::ScopedStatePtr StatePtr;

MultiGoalPlanResult
ApproachClustering::plan(const TreeScene &apples,
                         const moveit::core::RobotState &start_state,
                         const robowflex::SceneConstPtr &scene,
                         const robowflex::RobotConstPtr &robot,
                         PointToPointPlanner &point_to_point_planner) {

    std::vector<std::shared_ptr<DroneEndEffectorNearTarget>> goals;

    auto si = point_to_point_planner.getPlanner()->getSpaceInformation();

    for (const Apple &apple: apples.apples) {
        goals.push_back(std::make_shared<DroneEndEffectorNearTarget>(si, GOAL_END_EFFECTOR_RADIUS, apple.center));
    }

    auto ss = si->getStateSpace()->as<DroneStateSpace>();

    ompl::NearestNeighborsGNAT<StatePtr> gnat;
    gnat.setDistanceFunction([](const ompl::base::ScopedStatePtr &a, const ompl::base::ScopedStatePtr &b) {

        Eigen::Vector3d pos1(
                a->get()->as<DroneStateSpace::StateType>()->values /* base_link translation xyz are the first three values */);
        Eigen::Vector3d pos2(a->get()->as<DroneStateSpace::StateType>()->values);

        return (pos1 - pos2).squaredNorm();
    });

    for (const auto &goal: goals) {
        while (goal->getSamplesTried() < 100 && goal->getSamplesYielded() < 20) {
            StatePtr state(new ompl::base::ScopedState(si));

            goal->sampleGoal(state->get());
            if (si->isValid(state->get())) {
                gnat.add(state);
            }
        }
    }

    std::vector<StatePtr> states;
    gnat.list(states);
    boost::adjacency_list<> graph;

    std::vector<StatePtr> knn;

    for (const auto &st: states) {
        gnat.nearestK(st, initial_k, knn);
        for (const auto &item: knn) {
            if (si->checkMotion(st->get(), item->get())) {
                boost::add_edge(st, item, graph);
            }
        }
    }


}

std::string ApproachClustering::getName() {
    return "Approach Clustering";
}

ApproachClustering::ApproachClustering(size_t initialK) : initial_k(initialK) {}
