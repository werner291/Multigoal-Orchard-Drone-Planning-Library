#include <ompl/geometric/planners/prm/PRMstar.h>
#include "SingleGoalPlannerMethods.h"
#include "ManipulatorDroneMoveitPathLengthObjective.h"
#include "experiment_utils.h"

std::optional <ompl::geometric::PathGeometric>
SingleGoalPlannerMethods::state_to_goal(const ompl::base::State *a, const ompl::base::GoalPtr b) {
    auto objective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);
    auto prm = std::make_shared<ompl::geometric::PRMstar>(si);
    return planToGoal(*prm, objective, a, timePerAppleSeconds, true, b);
}

std::optional <ompl::geometric::PathGeometric>
SingleGoalPlannerMethods::state_to_state(const ompl::base::State *a, const ompl::base::State *b) {
    auto objective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);
    auto prm = std::make_shared<ompl::geometric::PRMstar>(si);
    return planFromStateToState(*prm, objective, a, b, timePerAppleSeconds);
}

const ompl::base::OptimizationObjectivePtr &SingleGoalPlannerMethods::getOptimizationObjective() const {
    return optimization_objective;
}
