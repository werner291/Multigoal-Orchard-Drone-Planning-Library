//
// Created by werner on 27-11-21.
//

#include "GATMatrix.h"

#include <utility>
#include <boost/graph/adjacency_list.hpp>
#include "approach_table.h"


MultiGoalPlanResult multigoal::LocalkGATPlanner::plan(GoalSet &goals, const ompl::base::State *start_state,
                                                      PointToPointPlanner &point_to_point_planner,
                                                      std::chrono::milliseconds time_budget) {

    const size_t APPROACHES = 10;

    const auto si = point_to_point_planner.getPlanner()->getSpaceInformation();
    auto gat = takeGoalSamples(si, goals, APPROACHES);

    struct StartState { /* Empty struct */ };

    typedef std::variant<StartState, Visitation> VertexProperty;
    typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperty, EdgeWeightProperty> Graph;
    typedef Graph::vertex_descriptor Vd;

    Graph g;

    ompl::NearestNeighborsGNAT<Vd> gnat;
    gnat.setDistanceFunction([&](const Vd &va, const Vd &vb) {
        VertexProperty a = g[va];
        VertexProperty b = g[vb];

        const auto sta = std::holds_alternative<StartState>(a) ? start_state : gat[std::get<Visitation>(
                a).target_idx][std::get<Visitation>(a).approach_idx]->get();
        const auto stb = std::holds_alternative<StartState>(b) ? start_state : gat[std::get<Visitation>(
                b).target_idx][std::get<Visitation>(b).approach_idx]->get();

        return si->distance(sta, stb);
    });

    for (size_t i = 0; i < goals.size(); ++i) {
        for (size_t j = 0; j < gat[i].size(); ++j) {

            Vd new_vd = boost::add_vertex(Visitation{i, j}, g);

            std::vector<Vd> knearest;
            gnat.nearestK(new_vd, APPROACHES * 10, knearest);

            for (Vd candidate_vd: knearest) {

                const auto sta = std::holds_alternative<StartState>(g[candidate_vd]) ? start_state :
                                 gat[std::get<Visitation>(g[candidate_vd]).target_idx][std::get<Visitation>(
                                         g[candidate_vd]).approach_idx]->get();

                auto plan_result = point_to_point_planner.planToOmplState(0.25, gat[i][j]->get(), sta);

                if (plan_result) { boost::add_edge(new_vd, candidate_vd, plan_result->length(), g); }
            }
        }
    }

    Vd start_vd;

    {
        start_vd = boost::add_vertex(StartState{}, g);

        std::vector<Vd> knearest;
        gnat.nearestK(start_vd, APPROACHES * 5, knearest);

        for (Vd candidate_vd: knearest) {

            const auto sta = std::holds_alternative<StartState>(g[candidate_vd]) ? start_state :
                             gat[std::get<Visitation>(g[candidate_vd]).target_idx][std::get<Visitation>(
                                     g[candidate_vd]).approach_idx]->get();

            auto plan_result = point_to_point_planner.planToOmplState(0.25, start_state, sta);

            if (plan_result) { boost::add_edge(start_vd, candidate_vd, plan_result->length(), g); }
        }
    }



//    std::vector<std::vector<double>> distance_matrix(g.vertex_set().size());
//    for (auto &item : distance_matrix) item.resize(g.vertex_set().size());
//
//    bool successful = boost::johnson_all_pairs_shortest_paths(g, distance_matrix);
//
//    assert(successful);

    return {};
}

std::string multigoal::LocalkGATPlanner::getName() {
    return "LocalkGATPlanner";
}

multigoal::LocalkGATPlanner::LocalkGATPlanner(GoalProjectionFn goalProjection, StateProjectionFn stateProjection) :
        goalProjection_(std::move(goalProjection)), stateProjection_(std::move(stateProjection)) {}

double multigoal::GATDistanceMatrix::lookup_start(const multigoal::Visitation &a) {
    return distances_from_start[a.approach_idx][a.target_idx];
}

double multigoal::GATDistanceMatrix::lookup(const multigoal::Visitation &a, const multigoal::Visitation &b) {
    return distances[a.approach_idx][b.approach_idx][a.target_idx][b.target_idx];
}

multigoal::GATDistanceMatrix
multigoal::GATDistanceMatrix::fullMatrix(const ompl::base::State *start_state, const multigoal::GoalApproachTable &gat,
                                         PointToPointPlanner &ptp) {

    GATDistanceMatrix mat;

    mat.distances.resize(gat.size());
    for (size_t target_idx_1 = 0; target_idx_1 < gat.size(); ++target_idx_1) {
        mat.distances[target_idx_1].resize(gat.size());

        for (size_t target_idx_2 = 0; target_idx_2 < gat.size(); ++target_idx_2) {

            mat.distances[target_idx_1][target_idx_2].resize(gat[target_idx_1].size());

            for (size_t approach_idx_1 = 0; approach_idx_1 < gat[target_idx_1].size(); ++approach_idx_1) {

                mat.distances[target_idx_1][target_idx_2][approach_idx_1].resize(gat[target_idx_1].size());

                for (size_t approach_idx_2 = 0; approach_idx_2 < gat[target_idx_1].size(); ++approach_idx_2) {

                    auto result = ptp.planToOmplState(0.25,
                                                      gat[target_idx_1][approach_idx_1]->get(),
                                                      gat[target_idx_2][approach_idx_2]->get());

                    mat.distances[target_idx_1][target_idx_2][approach_idx_1][approach_idx_2] = result
                                                                                                ? result->length()
                                                                                                : INFINITY;
                }
            }
        }
    }

    mat.distances_from_start.resize(gat.size());

    for (size_t target_idx_1 = 0; target_idx_1 < gat.size(); ++target_idx_1) {

        mat.distances_from_start[target_idx_1].resize(gat.size());

        for (size_t approach_idx_1 = 0; approach_idx_1 < gat[target_idx_1].size(); ++approach_idx_1) {

            auto result_fromstart = ptp.planToOmplState(0.25, start_state, gat[target_idx_1][approach_idx_1]->get());

            mat.distances_from_start[approach_idx_1] = result_fromstart ? result_fromstart->length() : INFINITY;

        }
    }

    return mat;
}
