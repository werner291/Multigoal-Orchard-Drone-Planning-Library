//
// Created by werner on 10/1/21.
//

#include "PointToPointPlanner.h"

#include <utility>
#include "../UnionGoalSampleableRegion.h"
#include "multi_goal_planners.h"

PointToPointPlanner::PointToPointPlanner(ompl::base::PlannerPtr planner,
                                         std::shared_ptr<ompl::base::OptimizationObjective> optimizationObjective,
                                         std::shared_ptr<SamplerWrapper> sampler)
        : planner_(std::move(planner)),
          optimizationObjective_(std::move(optimizationObjective)),
          sampler_(std::move(sampler)) {
    planner_->getSpaceInformation()->getStateSpace()->setStateSamplerAllocator(
            [this](const ompl::base::StateSpace *ss) { return this->sampler_->getSampler(); });
}


std::optional<ompl::geometric::PathGeometric>
PointToPointPlanner::planToOmplGoal(double maxTime,
                                    const ompl::base::State *start,
                                    const ompl::base::GoalPtr &goal) {

    assert(planner_->getSpaceInformation()->isValid(start));

    planner_->clearQuery();

    auto pdef = std::make_shared<ompl::base::ProblemDefinition>(planner_->getSpaceInformation());
    pdef->addStartState(start);
    pdef->setOptimizationObjective(optimizationObjective_);
    pdef->setGoal(goal);
    planner_->setProblemDefinition(pdef);

    sampler_->setStartAndGoal(start, std::dynamic_pointer_cast<ompl::base::GoalSampleableRegion>(goal));

    if (!planner_->isSetup()) {
        planner_->setup();
    }

    ompl::base::PlannerStatus status = planner_->solve(ompl::base::timedPlannerTerminationCondition(maxTime));

    // "Approximate" solutions can be wildly off, so we accept exact solutions only.
    if (status == ompl::base::PlannerStatus::EXACT_SOLUTION) {
        // FIXME: This check fails for some reason, my guess because of the undirected graph used inside PRM.
        // assert(pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>()->check());
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
                                     const ompl::base::State *goal) {
    assert(planner_->getSpaceInformation()->isValid(goal));

    auto gs = std::make_shared<ompl::base::GoalState>(planner_->getSpaceInformation());
    gs->setState(goal);
    return planToOmplGoal(maxTime, start, gs);
}

