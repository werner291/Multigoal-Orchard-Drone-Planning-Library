// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.
//

#ifndef NEW_PLANNERS_DYNAMICMULTIGOALPLANNEROMPLTOMOVEITADAPTER_H
#define NEW_PLANNERS_DYNAMICMULTIGOALPLANNEROMPLTOMOVEITADAPTER_H

#include <memory>
#include <ompl/base/SpaceInformation.h>
#include "../ompl_custom.h"
#include "../DynamicMultiGoalPlanner.h"
#include "../RobotPath.h"
#include "../AppleTreePlanningScene.h"
#include "../utilities/experiment_utils.h"

static const double APPLE_VISIT_MARGIN = 0.05;

/**
 * An adapter class that wraps a DynamicMultiGoalPlanner and converts back
 * and forth between OMPL types and MoveIt types.
 */
class DynamicMultiGoalPlannerOmplToMoveitAdapter {

	struct AppleHasher {
		std::size_t operator()(const Apple &apple) const {
			return std::hash<double>{}(apple.center.x()) ^ std::hash<double>{}(apple.center.y()) ^ std::hash<double>{}(apple.center.z());
		}
	};

	std::unordered_map<Apple, ompl::base::GoalPtr, AppleHasher> apple_to_ompl_goal {};

	const std::shared_ptr<DynamicMultiGoalPlanner> planner;
	ompl::base::SpaceInformationPtr si; ///< A pointer to the space information object.
	std::shared_ptr<DroneStateSpace> ss;
public:
	DynamicMultiGoalPlannerOmplToMoveitAdapter(const std::shared_ptr<DynamicMultiGoalPlanner> &planner,
											   ompl::base::SpaceInformationPtr si,
											   const std::shared_ptr<DroneStateSpace> &ss);

	/**
		 * @brief Plans a path to one of an initial set of goals, internally taking note of the set of goals such that
		 * subsequent calls to the replan* methods will plan to the new goals.
		 *
		 * @param start 			A pointer to the initial state.
		 * @param planning_scene 	The planning scene in which the planning should be done.
		 * @return An optional PathSegment containing a segment of the planned path to the next goal, or nullopt if the planner has decided to halt.
		 */
	[[nodiscard]] std::optional<RobotPath>
	plan(const moveit::core::RobotState &start_state, const AppleTreePlanningScene &planning_scene);


	/**
	 * @brief Yield the next segment of the path after a successful visit to a goal.
	 *
	 * @param current_state A pointer to the current state in the previously planned path.
	 * @param new_goals A vector of pointers to the new goals to add.
	 * @param goal_changes A GoalChanges object containing the new goals, visited goals, and removed goals.
	 * @return A PlanResult object containing the modified path and some statistics on the replanning process.
	 */
	[[nodiscard]] std::optional<RobotPath>
	replan_after_successful_visit(const moveit::core::RobotState &start_state,
								  const AppleTreePlanningScene &planning_scene);

	/**
	 * @brief Yield the next segment of the path after discovering a new goal.
	 *
	 * @param si A pointer to the OMPL space information.
	 * @param current_state A pointer to the current state in the previously planned path.
	 * @param new_goal A pointer to the new goal to add.
	 * @param goal_changes A GoalChanges object containing the new goals, visited goals, and removed goals.
	 * @return A PlanResult object containing the modified path and some statistics on the replanning process.
	 */
	[[nodiscard]] std::optional<RobotPath> replan_after_discovery(const moveit::core::RobotState &start_state,
																  const Apple &apple,
																  const PathInterrupt &interrupt,
																  const AppleTreePlanningScene &planning_scene);

	/**
	 * @brief Yield the next segment of the path after discovering that a previously-given goal does not exist.
	 *
	 * @param si A pointer to the OMPL space information.
	 * @param current_state A pointer to the current state in the previously planned path.
	 * @param removed_goal A pointer to the removed goal.
	 * @param goal_changes A GoalChanges object containing the new goals, visited goals, and removed goals.
	 * @return A PlanResult object containing the modified path and some statistics on the replanning process.
	 */
	[[nodiscard]] std::optional<RobotPath> replan_after_removal(const moveit::core::RobotState &start_state,
																  const Apple &apple,
																  const PathInterrupt &interrupt,
																  const AppleTreePlanningScene &planning_scene);

};


#endif //NEW_PLANNERS_DYNAMICMULTIGOALPLANNEROMPLTOMOVEITADAPTER_H
