#include <ompl/geometric/planners/prm/PRMstar.h>
#include "SingleGoalPlannerMethods.h"
#include "DronePathLengthObjective.h"
#include "experiment_utils.h"
#include "probe_retreat_move.h"
#include "DroneStateConstraintSampler.h"
#include "TimedCostConvergenceTerminationCondition.h"

std::optional<ompl::geometric::PathGeometric>
SingleGoalPlannerMethods::attempt_lucky_shot(const ompl::base::State *a, const ompl::base::GoalPtr &b) {

	// Convert the OMPL state to a MoveIt state
	moveit::core::RobotState robot_state(si->getStateSpace()->as<DroneStateSpace>()->getRobotModel());
	si->getStateSpace()->as<DroneStateSpace>()->copyToRobotState(robot_state, a);

	// Convert the goal
	auto goal = b->as<DroneEndEffectorNearTarget>();

	// Move the MoveIt state to the goal (this is safe since it's a copy)
	moveEndEffectorToGoal(robot_state, 0.01, goal->getTarget());

	// Convert the (now translated) MoveIt state back to an OMPL state
	ompl::base::ScopedState<> state(si->getStateSpace());
	si->getStateSpace()->as<DroneStateSpace>()->copyToOMPLState(state.get(), robot_state);

	// If the translated state is invalid, this fails.
	if (!si->isValid(state.get())) {
		return std::nullopt;
	}

	// Else, check if the motion is valid.
	if (si->checkMotion(a, state.get())) {

		// It is! We make a trivial path between the two states.
		ompl::geometric::PathGeometric path(si, a, state.get());

		return {path};

	} else {

		// Unfortunately the lucky shot failed, we'll have to do the expensive search.
		return {};

	}
}


std::optional<ompl::geometric::PathGeometric>
SingleGoalPlannerMethods::state_to_goal(const ompl::base::State *a, const ompl::base::GoalPtr b) {

	// Sanity check: start state should be valid
	assert(si->getStateValidityChecker()->isValid(a));

	// If asked to do so, see if the straight-line motion makes for a trivial solution.
	if (tryLuckyShots) {
		auto result = attempt_lucky_shot(a, b);
		if (result) {
			return result;
		}
	}

	// Set up the MakeshiftExponentialSamplerm allocator.
	// Yes, this mutates the SpaceInformation... We don't really have a choice, to my knowledge.
	if (useImprovisedSampler) {
		// When the sampler starts planning, this will be called by the planner.
		si->getStateSpace()->setStateSamplerAllocator([&](auto ss) {
			return std::make_shared<MakeshiftExponentialSampler>(
					ss,
					si->getStateSpace()->allocDefaultStateSampler(),
					a, // Here's the power of the MakeshiftExponentialSampler: it knows about the start and goal.
					std::dynamic_pointer_cast<ompl::base::GoalSampleableRegion>(b),
					MAKESHIFT_EXPONENTIAL_SAMPLER_DEVIATION
					);
		});
	} else {
		// It's supposed to do this anyway after planning, but I'm paranoid, so let's clear it anyway.
		si->getStateSpace()->clearStateSamplerAllocator();
	}


	auto ompl_planner = alloc(si);

	auto start_time = std::chrono::steady_clock::now();
	ompl::base::Planner &planner = *ompl_planner;

	auto pdef = std::make_shared<ompl::base::ProblemDefinition>(planner.getSpaceInformation());
	pdef->setOptimizationObjective(optimization_objective);
	pdef->addStartState(a);
	pdef->setGoal(b);

	planner.setProblemDefinition(pdef);

	std::optional<ompl::geometric::PathGeometric> result;

	auto ptc = useCostConvergence ? plannerOrTerminationCondition(ompl::base::timedPlannerTerminationCondition(
			timePerAppleSeconds), TimedConversionTerminationCondition(*pdef, ompl::time::seconds(0.025), true))
								  : ompl::base::timedPlannerTerminationCondition(timePerAppleSeconds);

	if (planner.solve(ptc) == ompl::base::PlannerStatus::EXACT_SOLUTION) {

		result = *pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();

		result = optimize(*result, optimization_objective, si);
	}

	if (useImprovisedSampler) {
		si->getStateSpace()->clearStateSamplerAllocator();
	}

	assert(!result || result->getStateCount() > 0);

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

Json::Value SingleGoalPlannerMethods::parameters() const {
	Json::Value params;
	params["timePerAppleSeconds"] = timePerAppleSeconds;
	params["ptp"] = alloc(si)->getName();
	params["useImprovisedSampler"] = useImprovisedSampler;
	params["tryLuckyShots"] = tryLuckyShots;
	params["useCostConvergence"] = useCostConvergence;
	return params;
}

