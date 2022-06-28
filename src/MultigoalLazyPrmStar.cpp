
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <range/v3/all.hpp>
#include "MultigoalLazyPrmStar.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace ranges;

class LazyPRMStarCustom : public og::LazyPRMstar {

};

struct AppleIdVertexPair {
    size_t apple_id;
    std::vector<LazyPRMStarCustom::Vertex> vertex;
};

/// For each apple, allocate a goal and and try to connect it to the PRM.
std::vector<AppleIdVertexPair> createGoalVertices(PRMCustom &prm,
                                                  const std::vector<ompl::base::GoalPtr> &apples,
                                                  PRMCustom::Vertex start,
                                                  const ompl::base::SpaceInformationPtr &si,
                                                  int samples_per_goal) {

    using namespace ranges;

    auto connect_coal = [&](const ompl::base::GoalPtr &goal) {
        return prm.tryConnectGoal(*goal->as<ompl::base::GoalSampleableRegion>(), samples_per_goal);
    };

    return apples
        | views::transform(connect_coal)
        | views::enumerate
        | views::transform([&](const auto &p) {
        return AppleIdVertexPair { p.first, p.second };
    }) | to_vector;

}

NewMultiGoalPlanner::PlanResult
MultigoalLazyPrmStar::plan(const ompl::base::SpaceInformationPtr &si,
                           const ompl::base::State *start,
                           const std::vector<ompl::base::GoalPtr> &goals) {

    std::cout << "Creating PRM ( budget: " << prm_build_time << "s)" << std::endl;
    auto prm = std::make_shared<LazyPRMStarCustom>(si);

    auto objective = std::make_shared<DronePathLengthObjective>(si);

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setOptimizationObjective(objective);
    prm->setProblemDefinition(pdef);

    prm->constructRoadmap(ob::timedPlannerTerminationCondition(prm_build_time));

    std::cout << "Inserting start/goal states into PRM..." << std::endl;

    auto start_state_node = prm->insert_state(start);

    auto goalVertices = createGoalVertices(*prm, goals, start_state_node, si, samplesPerGoal);

    auto vertices_only = goalVertices | views::transform([&](auto v) { return v.vertex; }) | to_vector;

    std::cout << "PRM with " << prm->getRoadmap().m_vertices.size() << " vertices and "
              << prm->getRoadmap().m_edges.size() << " edges" << std::endl;

    std::cout << "Solving TSP..." << std::endl;

    auto ordering = tsp_open_end_grouped(
            [&](auto pair) {
                auto path = prm->path_distance(start_state_node, goalVertices[pair.first].vertex[pair.second]);
                return path ? path->length() : std::numeric_limits<double>::infinity();
            },
            [&](auto pair_i, auto pair_j) {
                auto path = prm->path_distance(goalVertices[pair_i.first].vertex[pair_i.second], goalVertices[pair_j.first].vertex[pair_j.second]);
                return path ? path->length() : std::numeric_limits<double>::infinity();
            },
            goalVertices | views::transform([&](auto v) { return v.vertex.size(); }) | to_vector
    );

    std::cout << "Building final path" << std::endl;

    std::vector<og::PathGeometric> path_segments;

    path_segments.push_back(*(prm->path_distance(start_state_node,
                                                 goalVertices[ordering[0].first].vertex[ordering[0].second])->as<og::PathGeometric>()));

    for (size_t i = 1; i < ordering.size(); ++i) {
        path_segments.push_back(*(prm->path_distance(goalVertices[ordering[i-1].first].vertex[ordering[i-1].second],
                                                     goalVertices[ordering[i].first].vertex[ordering[i].second])->as<og::PathGeometric>()));
    }


    if (optimize_segments) {
        std::cout << "Optimizing..." << std::endl;
        path_segments |= actions::transform([&](auto &path) {
            return optimize(path, objective, si);
        });
    }

    PlanResult result;

    for (const auto &[segment, order_entry]: views::zip(path_segments, ordering)) {
        result.segments.push_back({
                                          goalVertices[order_entry.first].apple_id, segment
                                  });
    }

    return result;

}

Json::Value MultigoalLazyPrmStar::parameters() const {
    return Json::Value();
}

std::string MultigoalLazyPrmStar::name() const {
    return std::string();
}

MultigoalLazyPrmStar::MultigoalLazyPrmStar(double prmBuildTime) : prm_build_time(prmBuildTime) {}
