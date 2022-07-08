//
// Created by werner on 8-7-22.
//

#include "RoboTSP.h"
#include "traveling_salesman.h"
#include <range/v3/all.hpp>
#include <boost/graph/astar_search.hpp>

NewMultiGoalPlanner::PlanResult RoboTSP::plan(const ompl::base::SpaceInformationPtr &si, const ompl::base::State *start,
                                              const std::vector<ompl::base::GoalPtr> &goals) {

    // Sort the goals by the heuristic distances.

    auto ordering = tsp_open_end(
            [&](size_t i) -> double {
                return distance_heuristics->state_to_goal(start, goals[i]);
            },
            [&](size_t i, size_t j) -> double {
                return distance_heuristics->goal_to_goal(
                        goals[i].get(),
                        goals[j].get()
                );
            },
            goals.size()
    );

    // For all goals, take k valid goal samples.
    std::vector<std::vector<ompl::base::ScopedStatePtr>> goal_samples(goals.size());
    for (auto i: boost::irange<size_t>(0, goals.size())) {
        for (auto j: boost::irange<size_t>(0, k)) {
            auto st = std::make_shared<ompl::base::ScopedState<>>(si);
            goals[i]->as<ompl::base::GoalSampleableRegion>()->sampleGoal(st->get());
            goal_samples[i].push_back(st);
        }
    }

    // Build a graph of the goal samples, in order of the ordering.
    typedef boost::property<boost::edge_weight_t, int> EdgeWeightProperty;
    typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, boost::no_property, EdgeWeightProperty> Graph;
    typedef size_t Vertex;

    Graph graph;

    // First, from the start state to every goal sample for the first goal.
    for (auto i: boost::irange<size_t>(0, k)) {

        double distance = si->distance(start, goal_samples[ordering[0]][i]->get());

        boost::add_edge(0, i + 1, distance, graph);
    }

    // Then, for every goal sample, from the previous goal sample to the next in a full k*k fashion.
    for (auto i: boost::irange<size_t>(0, goals.size())) {
        for (auto j: boost::irange<size_t>(0, k)) {
            for (auto k: boost::irange<size_t>(0, k)) {

                double distance = si->distance(goal_samples[i][j]->get(), goal_samples[i][k]->get());

                boost::add_edge(i * k + j + 1, i * k + k + 1, distance, graph);
            }
        }
    }

    boost::vector_property_map<Vertex> prev(boost::num_vertices(graph));
    boost::vector_property_map<base::Cost> dist(boost::num_vertices(g_));
    boost::vector_property_map<base::Cost> rank(boost::num_vertices(g_));

    try {
        // Consider using a persistent distance_map if it's slow
        boost::astar_search(
                graph,
                start,
                [&](Vertex v) { return  },
                boost::predecessor_map(prev)
                        .distance_map(dist)
                        .rank_map(rank)
                        .distance_compare(
                                [this](base::Cost c1, base::Cost c2) { return opt_->isCostBetterThan(c1, c2); })
                        .distance_combine([this](base::Cost c1, base::Cost c2) { return opt_->combineCosts(c1, c2); })
                        .distance_inf(opt_->infiniteCost())
                        .distance_zero(opt_->identityCost())
                        .visitor(AStarGoalVisitor<Vertex>(goal)));
    }
    catch (AStarFoundGoal &) {
    }


    return PlanResult();
}

Json::Value RoboTSP::parameters() const {
    return Json::Value();
}

std::string RoboTSP::name() const {
    return std::string();
}
