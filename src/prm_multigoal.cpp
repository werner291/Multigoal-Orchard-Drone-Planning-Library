#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include "prm_multigoal.h"
#include "ManipulatorDroneMoveitPathLengthObjective.h"
#include "ompl_custom.h"
#include "planning_scene_diff_message.h"
#include "traveling_salesman.h"

#include <range/v3/view/filter.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/astar_search.hpp>

struct AStarFoundGoal
{
};  // exception for termination

// visitor that terminates when we find the goal
// V is the vertex type
template <typename V>
class AStarGoalVisitor : public boost::default_astar_visitor
{
public:
    AStarGoalVisitor(const V &goal) : goal_(goal)
    {
    }

    // G is the graph type
    template <typename G>
    void examine_vertex(const V &u, const G & /*unused*/)
    {
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

    Vertex tryConnectGoal(ob::GoalSampleableRegion& goal_region) {

        ob::State *st = si_->allocState();

        goal_region.sampleGoal(st);

        assert(si_->isValid(st));

        return addMilestone(st); // PRM takes ownership of the pointer

    }

    Vertex insert_state(const ob::State * st) {
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

    ob::State* get_state(Vertex v) {
        return stateProperty_[v];
    }

};

/// For each apple, allocate a goal and and try to connect it to the PRM.
/// Then, return only the ones where the connection succeeded.
/// Use rangev3 where possible
std::vector<PRMCustom::Vertex> createGoalVertices(PRMCustom &prm, const std::vector<Apple>& apples, PRMCustom::Vertex start, const ompl::base::SpaceInformationPtr &si) {

    using namespace ranges;

    std::vector<PRMCustom::Vertex> goal_vertices = apples
                                                   | views::transform([&](const Apple& apple) {

        DroneEndEffectorNearTarget goal(si, 0.05, apple.center);

        return prm.tryConnectGoal(goal);

    }) | to_vector;

    // Intentionally separating the transform and filter operations since prm gets mutated.

    return goal_vertices | ranges::views::filter([&](PRMCustom::Vertex v) {

        assert(prm.same_component(v, start) == prm.same_component(start, v));

        return prm.same_component(v, start);

    }) | to_vector;

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
PathMatrix pathMatrix(std::shared_ptr<PRMCustom> &prm, std::vector<PRMCustom::Vertex> &vertices) {

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
                                    const std::vector<PRMCustom::Vertex> &filtered,
                                    const std::vector<std::vector<ob::PathPtr>> &paths) {
    return tsp_open_end(
                [&](size_t i) {
                    return start_state_ompl.distance(prm->get_state(filtered[i]));
                },
                [&](size_t i, size_t j) {
                    return paths[i][j]->length();
                },
                filtered.size()
                );
}


std::vector<ob::PathPtr> getPathsFromStart(PRMCustom &prm, PRMCustom::Vertex start_state_node,
                            const std::vector<PRMCustom::Vertex> &goalVertices) {
    return goalVertices | views::transform([&](PRMCustom::Vertex goal_vertex) {
            return prm.path_distance(start_state_node, goal_vertex);
        }) | to_vector;
}

/**
 * \brief Given the ordering and path matrices, assemble the full path.
 */
og::PathGeometric buildFullPath(const ob::SpaceInformationPtr& si,
                                const std::vector<ob::PathPtr> &paths_from_start,
                                const PathMatrix &paths,
                                const std::vector<size_t> &ordering) {

    og::PathGeometric fullPath(si);

    fullPath = *paths_from_start[ordering[0]]->as<og::PathGeometric>();

    for (size_t i = 1; i < ordering.size(); ++i) {
        auto path = *paths[ordering[i - 1]][ordering[i]]->as<og::PathGeometric>();
        fullPath.append(path);
    }

    return fullPath;

}

RobotPath planByApples(const moveit::core::RobotState& start_state,
                       const planning_scene::PlanningSceneConstPtr& scene,
                       const std::vector<Apple>& apples) {

    auto state_space = std::make_shared<DroneStateSpace>(
            ompl_interface::ModelBasedStateSpaceSpecification(scene->getRobotModel(), "whole_body"), TRANSLATION_BOUND);
    auto si = initSpaceInformation(scene, scene->getRobotModel(), state_space);
    auto objective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);


    ob::ScopedState start_state_ompl(si);
    state_space->copyToOMPLState(start_state_ompl.get(), start_state);

    std::cout << "Creating PRM" << std::endl;
    auto prm = std::make_shared<PRMCustom>(si);

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
//    pdef->setOptimizationObjective(objective);
    prm->setProblemDefinition(pdef);

    prm->constructRoadmap(ob::timedPlannerTerminationCondition(1.0));

    std::cout << "Planning..." << std::endl;
    auto start_state_node = prm->insert_state(start_state_ompl.get());
    auto goalVertices = createGoalVertices(*prm, apples, start_state_node, si);

    auto paths_from_start = getPathsFromStart(*prm, start_state_node, goalVertices);
    auto paths = pathMatrix(prm, goalVertices);

    auto ordering = tsp_path_matrix(start_state_ompl, prm, goalVertices, paths);

    return omplPathToRobotPath(buildFullPath(si, paths_from_start, paths, ordering));

}