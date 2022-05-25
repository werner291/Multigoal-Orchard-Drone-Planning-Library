#include <ompl/geometric/planners/prm/PRMstar.h>
#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include "prm_multigoal.h"
#include "DronePathLengthObjective.h"
#include "ompl_custom.h"
#include "traveling_salesman.h"
#include "probe_retreat_move.h"

#include <range/v3/all.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace ranges;

typedef std::vector<std::vector<ob::PathPtr>> PathMatrix;

class PRMCustom : public og::PRMstar {

public:
    explicit PRMCustom(const ompl::base::SpaceInformationPtr &si) : PRMstar(si) {}

    std::vector<Vertex> tryConnectGoal(ob::GoalSampleableRegion &goal_region, size_t max_samples) {

        std::vector<Vertex> result;
        
        for (size_t i = 0; i < max_samples; ++i) {
            ob::State *st = si_->allocState();

            goal_region.sampleGoal(st);

            if (si_->isValid(st)) {
                result.push_back(addMilestone(st)); // PRM takes ownership of the pointer
            } else {
                si_->freeState(st);
                break;
            }
            
        }
        
        return result;
        
    }

    Vertex insert_state(const ob::State *st) {
        ob::State *st_copy = si_->allocState();
        si_->copyState(st_copy, st);
        return addMilestone(st_copy); // PRM takes ownership of the pointer
    }

    /**
     * Try to connect two vertices/states with a path. As an added optimization,
     * first check if the two vertices are even in the same component.
     *
     * @param u Start vertex
     * @param v End vertex
     * @return A path if one exists, or nullptr otherwise
     */
    ob::PathPtr path_distance(Vertex start, Vertex goal) {
        assert(same_component(start, goal));

        if (start == goal) {
            return std::make_shared<og::PathGeometric>(si_, stateProperty_[start]);
        } else {
            return constructSolution({start}, {goal});
        }
    }

    bool same_component(Vertex v, Vertex u) {
        graphMutex_.lock();
        bool same_component = sameComponent(v, u);
        graphMutex_.unlock();
        return same_component;
    }

};

struct AppleIdVertexPair {
    size_t apple_id;
    std::vector<PRMCustom::Vertex> vertex;
};

/// For each apple, allocate a goal and and try to connect it to the PRM.
/// Then, return only the ones where the connection succeeded.
/// Use rangev3 where possible
std::vector<AppleIdVertexPair> createGoalVertices(PRMCustom &prm,
                                                 const std::vector<ompl::base::GoalPtr> &apples,
                                                 PRMCustom::Vertex start,
                                                 const ompl::base::SpaceInformationPtr &si,
                                                 int samples_per_goal) {

    using namespace ranges;

    auto connect_coal = [&](const ompl::base::GoalPtr &goal) {
        return prm.tryConnectGoal(*goal->as<ompl::base::GoalSampleableRegion>(), samples_per_goal);
    };

    auto goal_vertices = apples
            | views::transform(connect_coal)
            | to_vector;

    // Intentionally separating the transform and filter operations since prm gets mutated.

    return goal_vertices
        | views::enumerate
        | views::transform([&](const auto &p) {

            auto vertices_reachable = p.second | views::filter([&](const auto &v) {
                return prm.same_component(start, v);
            }) | to_vector;

            return AppleIdVertexPair { p.first, vertices_reachable };

        })
        | to_vector;

}


NewMultiGoalPlanner::PlanResult MultigoalPrmStar::plan(
        const ompl::base::SpaceInformationPtr &si,
        const ompl::base::State *start,
        const std:: vector<ompl::base::GoalPtr> &goals) {

    std::cout << "Creating PRM ( budget: " << prm_build_time << "s)" << std::endl;
    auto prm = std::make_shared<PRMCustom>(si);

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

MultigoalPrmStar::MultigoalPrmStar(double prmBuildTime, size_t samplesPerGoal, bool optimizeSegments) : prm_build_time(
        prmBuildTime), samplesPerGoal(samplesPerGoal), optimize_segments(optimizeSegments) {}

Json::Value MultigoalPrmStar::parameters() const {
    Json::Value params;
    params["prm_build_time"] = prm_build_time;
    params["samples_per_goal"] = (int) samplesPerGoal;
    params["optimize_segments"] = optimize_segments;
    return params;
}

std::string MultigoalPrmStar::name() const {
    return "Multigoal PRM*";
}
