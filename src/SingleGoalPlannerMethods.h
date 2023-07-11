#ifndef NEW_PLANNERS_SINGLEGOALPLANNERMETHODS_H
#define NEW_PLANNERS_SINGLEGOALPLANNERMETHODS_H

static const double MAKESHIFT_EXPONENTIAL_SAMPLER_DEVIATION = 0.5;

#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/Goal.h>
#include <ompl/base/Planner.h>
#if __APPLE__
#include <json/json.h>
#else
#include <jsoncpp/json/value.h>
#endif
#include <optional>
#include <utility>
#include "InformedRobotStateSampler.h"

#include <ompl/geometric/planners/prm/PRM.h>

/**
 * A facade class that wraps up all the point-to-point planning machinery, offering simple methods to plan betweem:
 *  - two states
 *  - a state and a goal
 *
 *  Assumes that all goals are of type DroneEndEffectorNearTarget, and that we're using a DroneStateSpace
 *  for the state space.
 *
 *  Warning: this class modifies the SpaceInformation's StateSamplerAllocator!
 */
class SingleGoalPlannerMethods {

	/// The maximum amount of time to spend on a single planning attempt.
	const double timePerAppleSeconds;

	/// Link to OMPL's SpaceInformation God class.
	ompl::base::SpaceInformationPtr si;

	/// The optmization object (has not relaly been tested with anything other than path length)
	ompl::base::OptimizationObjectivePtr optimization_objective;

	/// An allocator function for the OMPL planner.
	ompl::base::PlannerAllocator alloc;

	/// Whether to use the MakeshiftExponentialSampler to sample states.
	bool useImprovisedSampler;

	/// Whether to explicitly try straight-line motions as paths first.
	bool tryLuckyShots;

	/// Whether to use the cost convergence termination condition.
	bool useCostConvergence;

protected:
	/**
	 * Attempt a straight-line motion from the start state to the goal state.
	 *
	 * @param a 	The start state.
	 * @param b 	The goal.
	 * @return 		A path if a straight-line motion is possible, otherwise std::nullopt.
	 *
	 */
	std::optional<ompl::geometric::PathGeometric>
	attempt_lucky_shot(const ompl::base::State *a, const ompl::base::GoalPtr &b);

public:

	/**
	 * Construct a SingleGoalPlannerMethods.
	 *
	 * @param planTimePerAppleSeconds 	The maximum amount of time to spend on a single planning attempt.
	 * @param si 						The OMPL SpaceInformation class. (Warning: State sampler allocator may be changed!)
	 * @param optimizationObjective 	The OMPL optimization objective.
	 * @param alloc 					The OMPL planner allocator.
	 * @param useImprovisedSampler 		Whether to use the MakeshiftExponentialSampler to sample states.
	 * @param tryLuckyShots 			Whether to explicitly try straight-line motions as paths first.
	 * @param useCostConvergence 		Whether to use the cost convergence termination condition.
	 */
	SingleGoalPlannerMethods(const double planTimePerAppleSeconds,
							 ompl::base::SpaceInformationPtr si,
							 ompl::base::OptimizationObjectivePtr optimizationObjective,
							 ompl::base::PlannerAllocator alloc,
							 bool useImprovisedSampler,
							 bool tryLuckyShots,
							 bool useCostConvergence)
			: timePerAppleSeconds(planTimePerAppleSeconds),
			  si(std::move(si)),
			  optimization_objective(std::move(optimizationObjective)),
			  alloc(std::move(alloc)),
			  useImprovisedSampler(useImprovisedSampler),
			  tryLuckyShots(tryLuckyShots),
			  useCostConvergence(useCostConvergence) {
	}

	/**
	 * Plan a path from a state to a goal.
	 *
	 * @param a 	The start state (assumed valid)
	 * @param b 	The goal
	 * @return 		A path from a to b, or std::nullopt if no path was found.
	 */
	std::optional<ompl::geometric::PathGeometric>
	state_to_goal(const ompl::base::State *a, const ompl::base::GoalPtr b);

	/**
	 * Plan a path from a state to another state.
	 *
	 * @param a 	The start state (assumed valid)
	 * @param b 	The goal state.
	 * @return 		A path from a to b, or std::nullopt if no path was found.
	 */
	std::optional<ompl::geometric::PathGeometric>
	state_to_state(const ompl::base::State *a, const ompl::base::State *b);

	/**
	 * A json representation of the SingleGoalPlannerMethods parameters.
	 */
	[[nodiscard]] Json::Value parameters() const;

};

#endif //NEW_PLANNERS_SINGLEGOALPLANNERMETHODS_H
