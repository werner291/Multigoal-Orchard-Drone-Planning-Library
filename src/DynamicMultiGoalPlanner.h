// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_DYNAMICMULTIGOALPLANNER_H
#define NEW_PLANNERS_DYNAMICMULTIGOALPLANNER_H

#include <vector>
#include <ompl/geometric/PathGeometric.h>
#include "planners/MultiGoalPlanner.h"

/**
 * @brief An abstract class for dynamic multi-goal planners.
 *
 * This abstract class defines the interface for dynamic multi-goal planners, which can plan a path to a set of goals
 * and then modify that path when new goals are added or removed. It provides three pure virtual methods: plan,
 * replan, and name.
 */
class DynamicMultiGoalPlanner {

public:
	using PlanResult = MultiGoalPlanner::PlanResult;

	/**
	 * @brief Plans a path to a set of goals.
	 *
	 * This method plans a path to a set of goals in a given planning scene, starting from a given initial state,
	 * using a given OMPL space information and planner termination condition. It returns a PlanResult object
	 * containing the planned path and some statistics on the planning process.
	 *
	 * @param si A pointer to the OMPL space information.
	 * @param start A pointer to the initial state.
	 * @param goals A vector of pointers to the goals.
	 * @param planning_scene The planning scene in which the planning should be done.
	 * @param ptc The OMPL planner termination condition.
	 * @return A PlanResult object containing the planned path and some statistics on the planning process.
	 */
	[[nodiscard]] virtual PlanResult plan(const ompl::base::SpaceInformationPtr &si,
										  const ompl::base::State *start,
										  const std::vector<ompl::base::GoalPtr> &goals,
										  const AppleTreePlanningScene &planning_scene,
										  ompl::base::PlannerTerminationCondition &ptc) = 0;

	/**
	 * A struct that represents the changes to the set of goals.
	 */
	struct GoalChanges {
		std::vector<ompl::base::GoalPtr> new_goals;
		std::vector<ompl::base::GoalPtr> visited_goals;
		std::vector<ompl::base::GoalPtr> removed_goals;
	};

	/**
	 * @brief Replans the path when goals are added or removed.
	 *
	 * This method modifies a previously planned path to a set of goals when new goals are added or removed,
	 * using a given OMPL space information and planner termination condition. It returns a PlanResult object
	 * containing the modified path and some statistics on the replanning process.
	 *
	 * @param si A pointer to the OMPL space information.
	 * @param current_state A pointer to the current state in the previously planned path.
	 * @param new_goals A vector of pointers to the new goals to add.
	 * @param goal_changes A GoalChanges object containing the new goals, visited goals, and removed goals.
	 * @param ptc The OMPL planner termination condition.
	 * @return A PlanResult object containing the modified path and some statistics on the replanning process.
	 */
	[[nodiscard]] virtual PlanResult replan(const ompl::base::SpaceInformationPtr &si,
											const ompl::base::State *current_state,
											const GoalChanges &goal_changes,
											const AppleTreePlanningScene &planning_scene,
											ompl::base::PlannerTerminationCondition &ptc) = 0;

};

#endif //NEW_PLANNERS_DYNAMICMULTIGOALPLANNER_H
