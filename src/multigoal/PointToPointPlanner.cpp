//
// Created by werner on 10/1/21.
//

#include "PointToPointPlanner.h"

#include <utility>
#include "../UnionGoalSampleableRegion.h"
#include "multi_goal_planners.h"


PointToPointPlanner::PointToPointPlanner(ompl::base::PlannerPtr planner,
                                         std::shared_ptr<ompl::base::OptimizationObjective> optimizationObjective)
        : planner_(std::move(planner)), optimizationObjective_(std::move(optimizationObjective)) {}


std::optional<ompl::geometric::PathGeometric>
PointToPointPlanner::planToOmplGoal(double maxTime, const ompl::base::State *start,
                                    const ompl::base::GoalPtr &goal) const {

    planner_->clearQuery();

    auto pdef = std::make_shared<ompl::base::ProblemDefinition>(planner_->getSpaceInformation());
    pdef->addStartState(start);
    pdef->setOptimizationObjective(optimizationObjective_);
    pdef->setGoal(goal);
    planner_->setProblemDefinition(pdef);

    // We explicitly do the setup beforehand to avoid counting it in the benchmarking.
    if (!planner_->isSetup()) planner_->setup();

    ompl::base::PlannerStatus status = planner_->solve(ompl::base::timedPlannerTerminationCondition(maxTime));

    // "Approximate" solutions can be wildly off, so we accept exact solutions only.
    if (status == ompl::base::PlannerStatus::EXACT_SOLUTION) {
        return {*pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>()};
    } else {
        return {};
    }
}

const ompl::base::PlannerPtr &PointToPointPlanner::getPlanner() const {
    return planner_;
}

const std::shared_ptr<ompl::base::OptimizationObjective> &PointToPointPlanner::getOptimizationObjective() const {
    return optimizationObjective_;
}

std::optional<ompl::geometric::PathGeometric>
PointToPointPlanner::planToOmplState(double maxTime, const ompl::base::State *start,
                                     const ompl::base::State *goal) const {
    auto gs = std::make_shared<ompl::base::GoalState>(planner_->getSpaceInformation());
    gs->setState(goal);
    return planToOmplGoal(maxTime, start, gs);
}
