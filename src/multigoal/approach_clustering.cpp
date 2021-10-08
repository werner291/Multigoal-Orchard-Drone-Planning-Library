//
// Created by werner on 30-09-21.
//

#include "approach_clustering.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <random>

using namespace multigoal;


MultiGoalPlanResult
ApproachClustering::plan(const TreeScene &apples,
                         const moveit::core::RobotState &start_state,
                         const robowflex::SceneConstPtr &scene,
                         const robowflex::RobotConstPtr &robot,
                         PointToPointPlanner &point_to_point_planner) {

    auto si = point_to_point_planner.getPlanner()->getSpaceInformation();
    auto goals = constructGoalRegions(apples, si);

    GoalApproachTable goal_samples = takeGoalSamples(si, goals, 10);


    random_initial_solution(goal_samples);



    //    auto ss = si->getStateSpace()->as<DroneStateSpace>();

//    ompl::NearestNeighborsGNAT<StatePtr> gnat;
//    gnat.setDistanceFunction([](const ompl::base::ScopedStatePtr &a, const ompl::base::ScopedStatePtr &b) {
//        return a->distance(b);
//    });



    //
//    std::vector<StatePtr> states;
//    gnat.list(states);
//    boost::adjacency_list<> graph;
//
//    std::vector<StatePtr> knn;
//
//    for (const auto &st: states) {
//        gnat.nearestK(st, initial_k, knn);
//        for (const auto &item: knn) {
//            if (si->checkMotion(st->get(), item->get())) {
//                boost::add_edge(st, item, graph);
//            }
//        }
//    }

}

std::vector<Visitation> ApproachClustering::random_initial_solution(const GoalApproachTable &goal_samples) {

    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()

    std::vector<Visitation> best_solution(goal_samples.size());
    for (size_t idx = 0; idx < goal_samples.size(); ++idx) {
        best_solution[idx] = {
                idx, std::uniform_int_distribution<size_t>(0, goal_samples[idx].size() - 1)(gen)
        };
    }
    std::shuffle(best_solution.begin(), best_solution.end(), gen);
    return best_solution;
}

GoalApproachTable
ApproachClustering::takeGoalSamples(const ompl::base::SpaceInformationPtr &si,
                                    const std::vector<GoalRegionPtr> &goals,
                                    int k) {

    GoalApproachTable goal_states(goals.size());

    for (size_t idx = 0; idx < goals.size(); ++idx) {
        const auto &goal = goals[idx];
        for (int i = 0; i < k; i++) {
            ob::ScopedStatePtr state(new ompl::base::ScopedState(si));

            goal->sampleGoal(state->get());
            if (si->isValid(state->get())) {
                goal_states[idx].push_back(state);
            }
        }
    }

    return goal_states;
}

std::vector<GoalRegionPtr>
ApproachClustering::constructGoalRegions(const TreeScene &apples, const ompl::base::SpaceInformationPtr &si) {
    std::vector<GoalRegionPtr> goals;
    for (const Apple &apple: apples.apples) {
        goals.push_back(std::make_shared<DroneEndEffectorNearTarget>(si, GOAL_END_EFFECTOR_RADIUS, apple.center));
    }
    return goals;
}

std::string ApproachClustering::getName() {
    return "Approach Clustering";
}

ApproachClustering::ApproachClustering(size_t initialK) : initial_k(initialK) {}

void ApproachClustering::keepBest(const ompl::base::OptimizationObjective &opt, GoalApproachTable &table, int keep_k) {

    for (auto &samples: table) {

        if (samples.size() > keep_k) {
            std::sort(samples.begin(), samples.end(), [&](const ob::ScopedStatePtr &a, const ob::ScopedStatePtr &b) {
                return opt.isCostBetterThan(opt.stateCost(a->get()), opt.stateCost(b->get()));
            });

            samples.resize(keep_k);

        }

    }


}
