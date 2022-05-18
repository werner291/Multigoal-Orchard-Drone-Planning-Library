#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include "prm_multigoal.h"
#include "DronePathLengthObjective.h"
#include "ompl_custom.h"
#include "planning_scene_diff_message.h"
#include "traveling_salesman.h"
#include "probe_retreat_move.h"

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/astar_search.hpp>
#include <range/v3/all.hpp>

struct AStarFoundGoal {
};  // exception for termination

// visitor that terminates when we find the goal
// V is the vertex type
template<typename V>
class AStarGoalVisitor : public boost::default_astar_visitor {
public:
    AStarGoalVisitor(const V &goal) : goal_(goal) {
    }

    // G is the graph type
    template<typename G>
    void examine_vertex(const V &u, const G & /*unused*/) {
        if (u == goal_)
            throw AStarFoundGoal();
    }

private:
    V goal_;
};

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

    ob::State *get_state(Vertex v) {
        return stateProperty_[v];
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
                                                 const std::vector<Apple> &apples,
                                                 PRMCustom::Vertex start,
                                                 const ompl::base::SpaceInformationPtr &si,
                                                 int samples_per_goal) {

    using namespace ranges;

    auto connect_coal = [&](const Apple &apple) {
        DroneEndEffectorNearTarget goal(si, 0.05, apple.center);
        return prm.tryConnectGoal(goal, samples_per_goal);
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

/**
 * Create the NxN path matrix for the given set of vertices.
 *
 * If all vertices are in the same component, then all paths will be valid.
 *
 * @param prm The PRM that contains the vertices
 * @param vertices The vertices to plan between.
 * @return A path matrix.
 */
PathMatrix pathMatrix(std::shared_ptr<PRMCustom> &prm, const std::vector<PRMCustom::Vertex> &vertices) {

    // Note: it should be possible to be faster than A* if we use an algorithm
    // that takes advantage of the fact that we're trying to compute multiple paths.

    return vertices | ranges::views::transform([&](PRMCustom::Vertex v) {
        return vertices | ranges::views::transform([&](PRMCustom::Vertex u) {
            return prm->path_distance(v, u);
        }) | ranges::to_vector;
    }) | ranges::to_vector;
}

std::vector<size_t> tsp_path_matrix(const ob::ScopedState<> &start_state_ompl,
                                    std::shared_ptr<PRMCustom> &prm,
                                    const std::vector<ob::PathPtr> &paths_from_start,
                                    const std::vector<std::vector<ob::PathPtr>> &paths) {

    assert(paths_from_start.size() == paths.size());

    return tsp_open_end(
            [&](size_t i) {return paths_from_start[i]->length();},
            [&](size_t i, size_t j) {return paths[i][j]->length();},
            paths_from_start.size()
    );
}


std::vector<ob::PathPtr> getPathsFromStart(PRMCustom &prm, PRMCustom::Vertex start_state_node,
                                           const std::vector<PRMCustom::Vertex> &goalVertices) {
    return goalVertices | views::transform([&](PRMCustom::Vertex goal_vertex) {
        return prm.path_distance(start_state_node, goal_vertex);
    }) | to_vector;
}

// Should consider using pointers here
std::vector<og::PathGeometric> ordering_to_path_segments(const std::vector<ob::PathPtr> &paths_from_start,
                                                         const PathMatrix &paths,
                                                         const std::vector<size_t> &ordering) {

    std::vector<og::PathGeometric> path_segments;

    path_segments.push_back(*paths_from_start[ordering[0]]->as<og::PathGeometric>());

    for (size_t i = 1; i < ordering.size(); ++i) {
        auto path = *paths[ordering[i - 1]][ordering[i]]->as<og::PathGeometric>();
        path_segments.push_back(path);
    }

    return path_segments;

}

MultiApplePlanResult
planByApples(const moveit::core::RobotState &start_state, const planning_scene::PlanningSceneConstPtr &scene,
             const std::vector<Apple> &apples, double prm_build_time, bool optimize_segments, size_t samplesPerGoal) {

    auto state_space = std::make_shared<DroneStateSpace>(
            ompl_interface::ModelBasedStateSpaceSpecification(scene->getRobotModel(), "whole_body"), TRANSLATION_BOUND);
    auto si = initSpaceInformation(scene, scene->getRobotModel(), state_space);
    auto objective = std::make_shared<DronePathLengthObjective>(si);

    ob::ScopedState start_state_ompl(si);
    state_space->copyToOMPLState(start_state_ompl.get(), start_state);

    std::cout << "Creating PRM" << std::endl;
    auto prm = std::make_shared<PRMCustom>(si);

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
//    pdef->setOptimizationObjective(objective);
    prm->setProblemDefinition(pdef);

    prm->constructRoadmap(ob::timedPlannerTerminationCondition(prm_build_time));

    std::cout << "Planning..." << std::endl;
    auto start_state_node = prm->insert_state(start_state_ompl.get());

    auto goalVertices = createGoalVertices(*prm, apples, start_state_node, si, samplesPerGoal);

    auto vertices_only = goalVertices | views::transform([&](auto v) { return v.vertex; }) | to_vector;

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

    std::vector<og::PathGeometric> path_segments;

    path_segments.push_back(*(prm->path_distance(start_state_node,
                                                 goalVertices[ordering[0].first].vertex[ordering[0].second])->as<og::PathGeometric>()));

    for (size_t i = 1; i < ordering.size(); ++i) {
        path_segments.push_back(*(prm->path_distance(goalVertices[ordering[i-1].first].vertex[ordering[i-1].second],
                                                     goalVertices[ordering[i].first].vertex[ordering[i].second])->as<og::PathGeometric>()));
    }

    if (optimize_segments) {
        path_segments |= actions::transform([&](auto &path) {
            return optimize(path, objective, si);
        });
    }

    auto moveit_segments = path_segments | views::transform([&](auto &path) {
        return omplPathToRobotPath(path);
    });

    std::vector<MultiApplePlanResult::AppleVisit> apple_visits;
    RobotPath fullPath;

    for (const auto &[segment, order_entry]: views::zip(path_segments, ordering)) {
        fullPath.append(omplPathToRobotPath(segment));
        apple_visits.push_back(MultiApplePlanResult::AppleVisit { goalVertices[order_entry.first].apple_id, fullPath.waypoints.size() - 1 });
    }

    return { fullPath, apple_visits };

}