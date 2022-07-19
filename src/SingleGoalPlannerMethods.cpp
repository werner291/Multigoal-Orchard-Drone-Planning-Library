#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/base/terminationconditions/CostConvergenceTerminationCondition.h>
#include "SingleGoalPlannerMethods.h"
#include "DronePathLengthObjective.h"
#include "experiment_utils.h"
#include "probe_retreat_move.h"
#include "DroneStateConstraintSampler.h"
#include "TimedCostConvergenceTerminationCondition.h"

std::optional<ompl::geometric::PathGeometric>
SingleGoalPlannerMethods::attempt_lucky_shot(const ompl::base::State *a, const ompl::base::GoalPtr &b) {
    moveit::core::RobotState robot_state(si->getStateSpace()->as<DroneStateSpace>()->getRobotModel());

    si->getStateSpace()->as<DroneStateSpace>()->copyToRobotState(robot_state, a);

    auto goal = b->as<DroneEndEffectorNearTarget>();

    moveEndEffectorToGoal(robot_state, 0.01, goal->getTarget());

    ompl::base::ScopedState<> state(si->getStateSpace());

    si->getStateSpace()->as<DroneStateSpace>()->copyToOMPLState(state.get(), robot_state);

    if (!si->isValid(state.get())) {
        return std::nullopt;
    }

    if (si->checkMotion(a, state.get())) {

        ompl::geometric::PathGeometric path(si, a, state.get());

        return {path};

    } else {

        return {};

    }
}


std::optional<ompl::geometric::PathGeometric>
SingleGoalPlannerMethods::state_to_goal(const ompl::base::State *a, const ompl::base::GoalPtr b) {

	assert(si->getStateValidityChecker()->isValid(a));

    if (tryLuckyShots) {
        auto result = attempt_lucky_shot(a, b);
        if (result) {
            return result;
        }
    }

    if (useImprovisedSampler) {
        si->getStateSpace()->setStateSamplerAllocator([&](auto ss) {
            return std::make_shared<MakeshiftExponentialSampler>(
                    ss,
                    si->getStateSpace()->allocDefaultStateSampler(),
                    a,
                    std::dynamic_pointer_cast<ompl::base::GoalSampleableRegion>(b),
                    0.5
            );
        });
    } else {
        // It's supposed to do this anyway after planning, but I'm paranoid, so let's clear it anyway.
        si->getStateSpace()->clearStateSamplerAllocator();
    }

    auto ompl_planner = alloc(si);

    auto start_time = std::chrono::steady_clock::now();
    ompl::base::Planner &planner = *ompl_planner;
    std::optional<ompl::geometric::PathGeometric> result1;
    auto pdef = std::make_shared<ompl::base::ProblemDefinition>(planner.getSpaceInformation());
    pdef->setOptimizationObjective(optimization_objective);
    pdef->addStartState(a);
    pdef->setGoal(b);

    planner.setProblemDefinition(pdef);

    std::optional<ompl::geometric::PathGeometric> result(si);

    auto ptc = useCostConvergence ? plannerOrTerminationCondition(
            ompl::base::timedPlannerTerminationCondition(timePerAppleSeconds),
            TimedConversionTerminationCondition(*pdef, ompl::time::seconds(0.025), true)
    ) : ompl::base::timedPlannerTerminationCondition(timePerAppleSeconds);

    if (planner.solve(ptc) ==
        ompl::base::PlannerStatus::EXACT_SOLUTION) {

        result = *pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();

        result = optimize(*result, optimization_objective, si);
    }

    if (useImprovisedSampler) {
        si->getStateSpace()->clearStateSamplerAllocator();
    }

    return result;
}

std::optional<ompl::geometric::PathGeometric>
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
    params["ptp"] = alloc(si)->getName();
    params["useImprovisedSampler"] = useImprovisedSampler;
    params["tryLuckyShots"] = tryLuckyShots;
    params["useCostConvergence"] = useCostConvergence;
    return params;
}

