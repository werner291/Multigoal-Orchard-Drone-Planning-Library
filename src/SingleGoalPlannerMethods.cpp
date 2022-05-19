#include <ompl/geometric/planners/prm/PRMstar.h>
#include "SingleGoalPlannerMethods.h"
#include "DronePathLengthObjective.h"
#include "experiment_utils.h"
#include "probe_retreat_move.h"

std::optional <ompl::geometric::PathGeometric>
SingleGoalPlannerMethods::state_to_goal(const ompl::base::State *a, const ompl::base::GoalPtr b) {
    auto prm = std::make_shared<ompl::geometric::PRMstar>(si);
    auto result = planToGoal(*prm, optimization_objective, a, timePerAppleSeconds, false, b);
    if (result) {
        *result = optimize(*result, optimization_objective, si);
    }
    return result;
}

std::optional <ompl::geometric::PathGeometric>
SingleGoalPlannerMethods::state_to_state(const ompl::base::State *a, const ompl::base::State *b) {
    auto objective = std::make_shared<DronePathLengthObjective>(si);
    auto prm = std::make_shared<ompl::geometric::PRMstar>(si);

    auto result = planFromStateToState(*prm, objective, a, b, timePerAppleSeconds);
    if (result) {
        *result = optimize(*result, optimization_objective, si);
    }
    return result;
}

const ompl::base::OptimizationObjectivePtr &SingleGoalPlannerMethods::getOptimizationObjective() const {
    return optimization_objective;
}
