#ifndef NEW_PLANNERS_POINTTOPOINTPLANNER_H
#define NEW_PLANNERS_POINTTOPOINTPLANNER_H

static const double GOAL_END_EFFECTOR_RADIUS = 0.2;

#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <robowflex_ompl/ompl_interface.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/robot.h>
#include <json/value.h>
#include "../procedural_tree_generation.h"
#include "../ompl_custom.h"
#include "../LeavesCollisionChecker.h"

struct PointToPointPlanResult {
    double solution_length{};
    robowflex::Trajectory point_to_point_trajectory;
    Eigen::Vector3d endEffectorTarget;
    size_t ith_target = 0;
};

/**
 * Represents some algorithm or strategy that, given a robot state and a (a set of) target point(s) in R^3, attempts
 * to plan a trajectory such that the end effector of the robot is within distance GOAL_END_EFFECTOR_RADIUS of
 * (at least one of) the goal(s).
 */
class PointToPointPlanner {

    /// The robot to plan for.
    const std::shared_ptr<robowflex::Robot> robot_;

    /// The OMPL planner to use. This may be a multi-query planner, the goals are cleared automatically.
    const ompl::base::PlannerPtr planner_;

    /// The optiization objective to use, incase of an optimizing planner.
    const std::shared_ptr<ompl::base::OptimizationObjective> optimizationObjective_;

    robowflex::Trajectory convertTrajectory(ompl::geometric::PathGeometric &path);

public:
    PointToPointPlanner(const ompl::base::PlannerPtr &planner,
                        const std::shared_ptr<ompl::base::OptimizationObjective> &optimizationObjective,
                        const std::shared_ptr<robowflex::Robot> robot);

    std::optional<PointToPointPlanResult> planPointToPoint(const moveit::core::RobotState &from_state,
                                                           const std::vector<Eigen::Vector3d> &targets,
                                                           double maxTime);

    std::optional<PointToPointPlanResult>
    planPointToPoint(const moveit::core::RobotState &from_state, const Eigen::Vector3d &target,
                     double maxTime);

    ompl::base::GoalPtr constructUnionGoal(const std::vector<Eigen::Vector3d> &targets);

    std::shared_ptr<ompl::base::ProblemDefinition>
    constructProblemDefinition(const moveit::core::RobotState &from_state, const ompl::base::GoalPtr &goal) const;
};


#endif //NEW_PLANNERS_POINTTOPOINTPLANNER_H
