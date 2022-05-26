#include <ompl/geometric/planners/prm/PRMstar.h>
#include "SingleGoalPlannerMethods.h"
#include "DronePathLengthObjective.h"
#include "experiment_utils.h"
#include "probe_retreat_move.h"

std::optional <ompl::geometric::PathGeometric>
SingleGoalPlannerMethods::state_to_goal(const ompl::base::State *a, const ompl::base::GoalPtr b) {
    auto ompl_planner = alloc(si);
    auto result = planToGoal(*ompl_planner, optimization_objective, a, timePerAppleSeconds, false, b);
    if (result) {
        *result = optimize(*result, optimization_objective, si);
    }
    return result;
}

std::optional <ompl::geometric::PathGeometric>
SingleGoalPlannerMethods::state_to_state(const ompl::base::State *a, const ompl::base::State *b) {
    auto ompl_planner = alloc(si);
    auto result = planFromStateToState(*ompl_planner, optimization_objective, a, b, timePerAppleSeconds);
    if (result) {
        *result = optimize(*result, optimization_objective, si);
    }
    return result;
}

const ompl::base::OptimizationObjectivePtr &SingleGoalPlannerMethods::getOptimizationObjective() const {
    return optimization_objective;
}

Json::Value SingleGoalPlannerMethods::parameters() const {
    Json::Value params;
    params["timePerAppleSeconds"] = timePerAppleSeconds;
    return params;
}
