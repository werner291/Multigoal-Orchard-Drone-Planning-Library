//
// Created by werner on 8-7-22.
//

#ifndef NEW_PLANNERS_ROBOTSP_H
#define NEW_PLANNERS_ROBOTSP_H



#include <boost/graph/astar_search.hpp>
#include "NewMultiGoalPlanner.h"
#include "DistanceHeuristics.h"

// See https://arxiv.org/abs/1709.09343
class RoboTSP : public NewMultiGoalPlanner {

    static const int INITIAL_STATE_VERTEX = 0;

    std::shared_ptr<OmplDistanceHeuristics> distance_heuristics;

    std::shared_ptr<SingleGoalPlannerMethods> methods;

    size_t k;

    // Build a graph of the goal samples, in order of the ordering.
    typedef boost::property<
            boost::edge_weight_t,
            double> EdgeWeightProperty;

    typedef boost::adjacency_list<
            boost::listS,
            boost::vecS,
            boost::undirectedS,
            boost::no_property,
            EdgeWeightProperty> Graph;

    using Vertex = boost::graph_traits<Graph>::vertex_descriptor;

    typedef boost::vector_property_map<RoboTSP::Vertex> PreviousVertexMap;
    typedef boost::vector_property_map<double> VertexDistanceMap;

    struct AStarFoundGoal : public std::exception {};

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedStructInspection"
    class AStarGoalVisitor : public boost::default_astar_visitor
    {
        Vertex first_goal_;

    public:

        explicit AStarGoalVisitor(Vertex firstGoal) : first_goal_(firstGoal) {}

        void examine_vertex(const Vertex &u, const Graph & /*unused*/) const;
    };
#pragma clang diagnostic pop


public:
    RoboTSP(const std::shared_ptr<OmplDistanceHeuristics> &distanceHeuristics,
            const std::shared_ptr<SingleGoalPlannerMethods> &methods, size_t k);

    PlanResult plan(const ompl::base::SpaceInformationPtr &si, const ompl::base::State *start,
                    const std::vector<ompl::base::GoalPtr> &goals) override;

    [[nodiscard]] Json::Value parameters() const override;

    [[nodiscard]] std::string name() const override;

    std::vector<size_t>
    heuristic_preorder(const ompl::base::State *start, const std::vector<ompl::base::GoalPtr> &goals) const;

    [[nodiscard]] std::vector<std::vector<ompl::base::ScopedStatePtr>>
    take_goal_samples(const ompl::base::SpaceInformationPtr &si, const std::vector<ompl::base::GoalPtr> &goals) const;

    boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, boost::no_property, EdgeWeightProperty>
    build_graph(const ompl::base::SpaceInformationPtr &si, const ompl::base::State *start,
                const std::vector<ompl::base::GoalPtr> &goals, const std::vector<size_t> &ordering,
                const std::vector<std::vector<ompl::base::ScopedStatePtr>> &goal_samples) const;

    [[nodiscard]] Vertex graph_vertex(size_t step_i, size_t sample_i) const;

    const ompl::base::State * state_for_vertex(
            Vertex v,
            const ompl::base::State *start,
            const std::vector<std::vector<ompl::base::ScopedStatePtr>> &goal_samples,
            const std::vector<size_t> &ordering) const;

    [[nodiscard]] std::tuple<
            PreviousVertexMap,
            VertexDistanceMap>
    run_search(const std::vector<ompl::base::GoalPtr> &goals, const RoboTSP::Graph &graph) const;

    [[nodiscard]] Vertex find_optimal_final_state_vertex(const std::vector<ompl::base::GoalPtr> &goals, const VertexDistanceMap& dist) const;

    std::vector<const ompl::base::State *>
    retraceWithStates(const ompl::base::State *start,
                               const std::vector<size_t> &ordering,
                               const std::vector<std::vector<ompl::base::ScopedStatePtr>> &goal_samples,
                               const PreviousVertexMap& prev,
                               Vertex goal_vertex) const ;

    PlanResult planFullPath(const std::vector<size_t> &ordering, const std::vector<const ompl::base::State *> &path);

    void
    build_start_state_edges(const ompl::base::SpaceInformationPtr &si, const ompl::base::State *start, Graph &graph,
                            const std::vector<size_t> &ordering,
                            const std::vector<std::vector<ompl::base::ScopedStatePtr>> &goal_samples) const;

    void buildEdgesBetweenLayers(const ompl::base::SpaceInformationPtr &si, const std::vector<size_t> &ordering,
                                 const std::vector<std::vector<ompl::base::ScopedStatePtr>> &goal_samples, Graph &graph,
                                 Vertex i) const;
};


#endif //NEW_PLANNERS_ROBOTSP_H
