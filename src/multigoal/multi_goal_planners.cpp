#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <robowflex_library/trajectory.h>

#include <random>
#include "multi_goal_planners.h"
#include "../ompl_custom.h"
#include "../procedural_tree_generation.h"
#include "../json_utils.h"
#include "../UnionGoalSampleableRegion.h"
#include "knn.h"
#include "uknn.h"
#include "random_order.h"

std::shared_ptr<ompl::base::SpaceInformation>
initSpaceInformation(const robowflex::SceneConstPtr &scene, const robowflex::RobotConstPtr &robot,
                     std::shared_ptr<DroneStateSpace> &state_space);


void extendTrajectory(robowflex::Trajectory &full_trajectory, robowflex::Trajectory &extension) {
    for (size_t i = 0; i < extension.getNumWaypoints(); i++) {
        full_trajectory.addSuffixWaypoint(extension.getTrajectory()->getWayPoint(i));
    }
}


std::optional<PointToPointPlanResult>
planPointToPoint(const robowflex::RobotConstPtr &robot, ompl::base::Planner &planner, const ompl::base::GoalPtr &goal,
                 const moveit::core::RobotState &from_state, double max_time) {

    std::optional<PointToPointPlanResult> result;

    robowflex::Trajectory trajectory(robot, "whole_body");

    auto state_space = planner.getSpaceInformation()->getStateSpace()->as<DroneStateSpace>();

    ompl::base::ScopedState start(planner.getSpaceInformation());
    state_space->copyToOMPLState(start.get(), from_state);

    auto pdef = std::make_shared<ompl::base::ProblemDefinition>(planner.getSpaceInformation());
    pdef->addStartState(start.get());

    pdef->setGoal(goal);

    planner.setProblemDefinition(pdef);
    if (!planner.isSetup()) planner.setup(); // We explicitly do the setup beforehand to avoid counting it in the benchmarking.

    std::chrono::steady_clock::time_point pre_solve = std::chrono::steady_clock::now();
    ompl::base::PlannerStatus status = planner.solve(ompl::base::timedPlannerTerminationCondition(
            max_time));
    std::chrono::steady_clock::time_point post_solve = std::chrono::steady_clock::now();

    long elapsed_millis = std::chrono::duration_cast<std::chrono::milliseconds>(
            (post_solve - pre_solve)).count();

    ompl::geometric::PathSimplifier ps(planner.getSpaceInformation());

    if (status == ompl::base::PlannerStatus::EXACT_SOLUTION) {

        auto path = pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();

//        ps.shortcutPath(*path, 50);
//        ps.smoothBSpline(*path);

        moveit::core::RobotState st(robot->getModelConst());

//        assert(planner.getSpaceInformation()->checkMotion(path->getStates(),3));

        for (auto state: path->getStates()) {
            state_space->copyToRobotState(st, state);
            trajectory.addSuffixWaypoint(st);
        }

        result = {
                .solution_length = path->length(),
                .point_to_point_trajectory = trajectory
        };
    } else {
        result = {};
        std::cout << "Apple unreachable" << std::endl;
    }

    planner.clearQuery();

    return result;
}

