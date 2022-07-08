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
    auto ordering = heuristic_preorder(start, goals);

    // For all goals, take k valid goal samples.
    auto goal_samples = take_goal_samples(si, goals);

    Graph graph = build_graph(si, start, goals, ordering, goal_samples);

    const auto&[prev, dist] = run_search(goals, graph);
    Vertex goal_vertex = find_optimal_final_state_vertex(goals, dist);

    // Retrace the path.
    std::vector<const ompl::base::State *> path = retraceWithStates(start, ordering, goal_samples, prev, goal_vertex);

    // Then, use the point-to-point planner to plan the full path.
    return planFullPath(ordering, path);
}

NewMultiGoalPlanner::PlanResult
RoboTSP::planFullPath(const std::vector<size_t> &ordering, const std::vector<const ompl::base::State *> &path) {
    PlanResult total_result;

    for (auto i : boost::irange<size_t>(0, path.size() - 1)) {
        auto result = methods->state_to_state(path[i], path[i + 1]);
        if (result) {
            total_result.segments.push_back({
                ordering[i],
                *result
            });
        } else {
            std::cout << "Failed to plan path from " << i << " to " << i + 1 << std::endl;
        }
    }
    return total_result;
}

std::vector<const ompl::base::State *>
RoboTSP::retraceWithStates(const ompl::base::State *start,
                           const std::vector<size_t> &ordering,
                           const std::vector<std::vector<ompl::base::ScopedStatePtr>> &goal_samples,
                           const RoboTSP::PreviousVertexMap& prev,
                           RoboTSP::Vertex goal_vertex) const {

    std::vector<const ompl::base::State*> path;
    for (Vertex v = goal_vertex; v != 0; v = prev[v]) {
        path.push_back(state_for_vertex(v, start, goal_samples, ordering));
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

RoboTSP::Vertex RoboTSP::find_optimal_final_state_vertex(const std::vector<ompl::base::GoalPtr> &goals,
                                                         const VertexDistanceMap& dist) const {// Find the goal vertex with smallest distance.
    Vertex goal_vertex;
    double min_distance = INFINITY;

    for (Vertex i : boost::irange<size_t>(graph_vertex(goals.size() - 1, 0),graph_vertex(goals.size() - 1, k))) {
        if (dist[i] < min_distance) {
            min_distance = dist[i];
            goal_vertex = i;
        }
    }
    return goal_vertex;
}

std::tuple<RoboTSP::PreviousVertexMap, RoboTSP::VertexDistanceMap>
RoboTSP::run_search(const std::vector<ompl::base::GoalPtr> &goals, const RoboTSP::Graph &graph) const {

    boost::vector_property_map<double> rank(boost::num_vertices(graph));
    boost::vector_property_map<Vertex> prev;
    boost::vector_property_map<double> dist;

    try {
        // Consider using a persistent distance_map if it's slow
        boost::astar_search(
                graph,
                0,
                [&](Vertex v) { return 0.0; },
                boost::predecessor_map(prev)
                    .distance_map(dist)
                    .rank_map(rank)
                    .distance_compare([](double c1, double c2) { return c1 < c2; })
                    .distance_combine([](double c1, double c2) { return c1 + c2; })
                    .distance_inf(INFINITY)
                    .distance_zero(0.0)
                    .visitor(AStarGoalVisitor((goals.size() - 1) * k + 1)));
    }
    catch (AStarFoundGoal &) {
    }

    return std::make_tuple(prev, dist);
}

RoboTSP::Graph RoboTSP::build_graph(const ompl::base::SpaceInformationPtr &si, const ompl::base::State *start,
                           const std::vector<ompl::base::GoalPtr> &goals, const std::vector<size_t> &ordering,
                           const std::vector<std::vector<ompl::base::ScopedStatePtr>> &goal_samples) const {
    Graph graph;

    // First, from the start state to every goal sample for the first goal.
    build_start_state_edges(si, start, graph, ordering, goal_samples);

    // Then, for every goal sample, from the previous goal sample to the next in a full k*k fashion.
    for (auto i: boost::irange<size_t>(0, goals.size()-1)) {
        buildEdgesBetweenLayers(si, ordering, goal_samples, graph, i);
    }

    return graph;
}

void RoboTSP::buildEdgesBetweenLayers(const ompl::base::SpaceInformationPtr &si, const std::vector<size_t> &ordering,
                                      const std::vector<std::vector<ompl::base::ScopedStatePtr>> &goal_samples,
                                      RoboTSP::Graph &graph, RoboTSP::Vertex i) const {
    for (auto sample_i: boost::irange<size_t>(0, k)) {
        for (auto sample_j: boost::irange<size_t>(0, k)) {
            double distance = si->distance(goal_samples[ordering[i]][sample_i]->get(), goal_samples[ordering[i+1]][sample_j]->get());

            boost::add_edge(graph_vertex(i, sample_i), graph_vertex(i + 1, sample_j), distance, graph);
        }
    }
}

void RoboTSP::build_start_state_edges(const ompl::base::SpaceInformationPtr &si, const ompl::base::State *start,
                                      RoboTSP::Graph &graph, const std::vector<size_t> &ordering,
                                      const std::vector<std::vector<ompl::base::ScopedStatePtr>> &goal_samples) const {
    for (auto i: boost::irange<size_t>(0, k)) {
        double distance = si->distance(start, goal_samples[ordering[0]][i]->get());

        boost::add_edge(INITIAL_STATE_VERTEX, graph_vertex(0, i), distance, graph);
    }
}

std::vector<std::vector<ompl::base::ScopedStatePtr>>
RoboTSP::take_goal_samples(const ompl::base::SpaceInformationPtr &si, const std::vector<ompl::base::GoalPtr> &goals) const {
    std::vector<std::vector<ompl::base::ScopedStatePtr>> goal_samples(goals.size());
    for (auto i: boost::irange<size_t>(0, goals.size())) {
        for (auto j: boost::irange<size_t>(0, k)) {
            auto st = std::make_shared<ompl::base::ScopedState<>>(si);
            goals[i]->as<ompl::base::GoalSampleableRegion>()->sampleGoal(st->get());
            goal_samples[i].push_back(st);
        }
    }
    return goal_samples;
}

std::vector<size_t>
RoboTSP::heuristic_preorder(const ompl::base::State *start, const std::vector<ompl::base::GoalPtr> &goals) const {
    return tsp_open_end(
                [&](size_t i) -> double {
                    return distance_heuristics->state_to_goal(start, goals[i].get());
                },
                [&](size_t i, size_t j) -> double {
                    return distance_heuristics->goal_to_goal(
                            goals[i].get(),
                            goals[j].get()
                    );
                },
                goals.size()
        );
}

Json::Value RoboTSP::parameters() const {
    Json::Value result;
    result["k"] = k;
    result["methods"] = methods->parameters();
    return result;
}

std::string RoboTSP::name() const {
    return "RoboTSP";
}

RoboTSP::Vertex RoboTSP::graph_vertex(size_t step_i, size_t sample_i) const {
    return step_i * k + sample_i + 1;
}

const ompl::base::State *RoboTSP::state_for_vertex(RoboTSP::Vertex v, const ompl::base::State *start,
                                                   const std::vector<std::vector<ompl::base::ScopedStatePtr>> &goal_samples,
                                                   const std::vector<size_t> &ordering) const {
    return v == INITIAL_STATE_VERTEX ? start : goal_samples[ordering[(v-1) / k]][(v-1) % k]->get();
}

RoboTSP::RoboTSP(const std::shared_ptr<OmplDistanceHeuristics> &distanceHeuristics,
                 const std::shared_ptr<SingleGoalPlannerMethods> &methods, size_t k) : distance_heuristics(
        distanceHeuristics), methods(methods), k(k) {}

void RoboTSP::AStarGoalVisitor::examine_vertex(const RoboTSP::Vertex &u, const RoboTSP::Graph &) const {
    if (u >= first_goal_)
        throw AStarFoundGoal();
}
