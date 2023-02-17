// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_DYNAMICMULTIGOALPLANNER_H
#define NEW_PLANNERS_DYNAMICMULTIGOALPLANNER_H

#include <vector>
#include <ompl/geometric/PathGeometric.h>
#include "planners/MultiGoalPlanner.h"
#include "PathInterrupt.h"

/**
 * @brief An abstract class for dynamic multi-goal planners.
 *
 * This abstract class defines the interface for dynamic multi-goal planners, which can plan a path to a set of goals
 * and then modify that path when new goals are added or removed.
 *
 * The path is returned one segment at a time, each time to a new goal.
 */
class DynamicMultiGoalPlanner {

public:
	using PathSegment = MultiGoalPlanner::PathSegment;

	/**
	 * @brief Plans a path to one of an initial set of goals, internally taking note of the set of goals such that
	 * subsequent calls to the replan* methods will plan to the new goals.
	 *
	 * @param si 				A pointer to the OMPL space information.
	 * @param start 			A pointer to the initial state.
	 * @param goals 			A vector of pointers to the goals.
	 * @param planning_scene 	The planning scene in which the planning should be done.
	 * @return An optional PathSegment containing a segment of the planned path to the next goal, or nullopt if the planner has decided to halt.
	 */
	[[nodiscard]] virtual std::optional<PathSegment> plan(const ompl::base::SpaceInformationPtr &si,
														  const ompl::base::State *start,
														  const std::vector<ompl::base::GoalPtr> &goals,
														  const AppleTreePlanningScene &planning_scene) = 0;

	/**
	 * @brief Yield the next segment of the path after a successful visit to a goal.
	 *
	 * @param si A pointer to the OMPL space information.
	 * @param current_state A pointer to the current state in the previously planned path.
	 * @param new_goals A vector of pointers to the new goals to add.
	 * @param goal_changes A GoalChanges object containing the new goals, visited goals, and removed goals.
	 * @return A PlanResult object containing the modified path and some statistics on the replanning process.
	 */
	[[nodiscard]] virtual std::optional<PathSegment>
	replan_after_successful_visit(const ompl::base::SpaceInformationPtr &si,
								  const ompl::base::State *current_state,
								  const ompl::base::GoalPtr &visited_goal,
								  const AppleTreePlanningScene &planning_scene) = 0;

	/**
	 * @brief Yield the next segment of the path after discovering a new goal.
	 *
	 * @param si A pointer to the OMPL space information.
	 * @param current_state A pointer to the current state in the previously planned path.
	 * @param new_goal A pointer to the new goal to add.
	 * @param goal_changes A GoalChanges object containing the new goals, visited goals, and removed goals.
	 * @return A PlanResult object containing the modified path and some statistics on the replanning process.
	 */
	[[nodiscard]] virtual std::optional<PathSegment> replan_after_discovery(const ompl::base::SpaceInformationPtr &si,
																			const ompl::base::State *current_state,
																			const ompl::base::GoalPtr &new_goal,
																			const PathInterrupt &interrupt,
																			const AppleTreePlanningScene &planning_scene) = 0;

	/**
	 * @brief Yield the next segment of the path after discovering that a goal has been removed.
	 *
	 * @param si A pointer to the OMPL space information.
	 * @param current_state A pointer to the current state in the previously planned path.
	 * @param removed_goal A pointer to the removed goal.
	 * @param goal_changes A GoalChanges object containing the new goals, visited goals, and removed goals.
	 * @return A PlanResult object containing the modified path and some statistics on the replanning process.
	 */
	[[nodiscard]] virtual std::optional<PathSegment> replan_after_removal(const ompl::base::SpaceInformationPtr &si,
																		  const ompl::base::State *current_state,
																		  const ompl::base::GoalPtr &removed_goal,
																		  const PathInterrupt &interrupt,
																		  const AppleTreePlanningScene &planning_scene) = 0;

};

#endif //NEW_PLANNERS_DYNAMICMULTIGOALPLANNER_H
