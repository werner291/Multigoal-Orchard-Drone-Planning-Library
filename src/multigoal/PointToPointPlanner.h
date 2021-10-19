#ifndef NEW_PLANNERS_POINTTOPOINTPLANNER_H
#define NEW_PLANNERS_POINTTOPOINTPLANNER_H


#include <ompl/geometric/PathGeometric.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/Planner.h>

/**
 * Represents some algorithm or strategy that, given a robot state and a (a set of) target point(s) in R^3, attempts
 * to plan a trajectory such that the end effector of the robot is within distance GOAL_END_EFFECTOR_RADIUS of
 * (at least one of) the goal(s).
 */
class PointToPointPlanner {

    /// The OMPL planner to use. This may be a multi-query planner, the goals are cleared automatically.
    const ompl::base::PlannerPtr planner_;

    /// The optimization objective to use, incase of an optimizing planner.
    const std::shared_ptr<ompl::base::OptimizationObjective> optimizationObjective_;

public:
    [[nodiscard]] const ompl::base::PlannerPtr &getPlanner() const;

    [[nodiscard]] const std::shared_ptr<ompl::base::OptimizationObjective> &getOptimizationObjective() const;

    PointToPointPlanner(ompl::base::PlannerPtr planner,
                        std::shared_ptr<ompl::base::OptimizationObjective> optimizationObjective);

    [[nodiscard]] std::optional<ompl::geometric::PathGeometric>
    planToOmplGoal(double maxTime, const ompl::base::State *start, const ompl::base::GoalPtr &goal) const;

    [[nodiscard]] std::optional<ompl::geometric::PathGeometric>
    planToOmplState(double maxTime, const ompl::base::State *start, const ompl::base::State *goal) const;
};


#endif //NEW_PLANNERS_POINTTOPOINTPLANNER_H