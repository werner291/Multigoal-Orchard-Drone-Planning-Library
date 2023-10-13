// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 15-2-23.
//

#ifndef NEW_PLANNERS_MAKESHIFTPRMAPPROACHPLANNINGMETHODS_H
#define NEW_PLANNERS_MAKESHIFTPRMAPPROACHPLANNINGMETHODS_H

#include "ApproachPlanning.h"
#include "../../SingleGoalPlannerMethods.h"

/**
 * @class MakeshiftPrmApproachPlanningMethods
 * @brief A class that implements the ApproachPlanningMethods for OMPL-based planning.
 *
 * This class specifically uses Probabilistic Roadmap Methods (PRM) in its planning approach, with an optional mechanism
 * for trying straight-line motions before invoking the actual planning algorithm.
 *
 * It uses a mutex to ensure thread safety across multiple calls to its planning methods,
 * so make sure to use multiple instances for multi-threaded applications (this is due to a limitation in OMPL/MoveIt).
 *
 * @tparam ShellPoint Type of the points that define the "shell" around the goal region, to be used in the planning process.
 */
template<typename ShellPoint>
class MakeshiftPrmApproachPlanningMethods : public ApproachPlanningMethods<ShellPoint> {
	/// A mutex to ensure thread-safety across multiple calls to the planning methods.
	mutable std::mutex mutex;

	/// The OMPL-based single goal planner method instance.
	mutable std::shared_ptr<SingleGoalPlannerMethods> single_goal_planner_methods;

	/// The SpaceInformation instance that encapsulates the problem space for the OMPL planner.
	const ompl::base::SpaceInformationPtr si;

public:
	/**
	 * Constructor
	 *
	 * @param si    The OMPL SpaceInformation instance that encapsulates the problem space for the planner.
	 * @param t_max The maximum time allowed for each planning attempt.
	 */
	explicit MakeshiftPrmApproachPlanningMethods(ompl::base::SpaceInformationPtr si, double t_max = 1.0);

	/**
	 * Method to compute a path from a starting state to a goal state.
	 *
	 * @param start The starting state.
	 * @param shell The shell space that the planner can use to assist in finding a path.
	 *
	 * @return An optional OmplApproachPath instance. If a path was found, it is returned; otherwise, an empty optional is returned.
	 */
	std::optional<OmplApproachPath<ShellPoint>>
	approach_path(const ompl::base::State *start, const OmplShellSpace<ShellPoint> &shell) const override;

	/**
	 * Method to compute a path from a goal state to a starting state.
	 *
	 * @param goal The goal state.
	 * @param shell The shell space that the planner can use to assist in finding a path.
	 *
	 * @return An optional OmplApproachPath instance. If a path was found, it is returned; otherwise, an empty optional is returned.
	 */
	std::optional<OmplApproachPath<ShellPoint>>
	approach_path(const ompl::base::GoalPtr &goal, const OmplShellSpace<ShellPoint> &shell) const override;

};



#endif //NEW_PLANNERS_MAKESHIFTPRMAPPROACHPLANNINGMETHODS_H
