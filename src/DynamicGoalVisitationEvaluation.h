// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_DYNAMICGOALVISITATIONEVALUATION_H
#define NEW_PLANNERS_DYNAMICGOALVISITATIONEVALUATION_H

#include <range/v3/view/iota.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/zip.hpp>
#include "DynamicMultiGoalPlanner.h"
#include "utilities/goal_events.h"
#include "utilities/experiment_utils.h"
#include "planners/DynamicMultiGoalPlannerOmplToMoveitAdapter.h"

/**
 * @brief A class representing an evaluation framework for a given planner.
 *
 * The DynamicGoalVisitationEvaluation class encapsulates the evaluation process for a given planner,
 * allowing other code to focus on gathering statistics and/or visualization as needed.
 */
class DynamicGoalVisitationEvaluation {

	using Planner = DynamicMultiGoalPlannerOmplToMoveitAdapter;

	/// A pointer to the planner being evaluated.
	std::shared_ptr<Planner> planner;

	/// The initial state of the robot.
	moveit::core::RobotState last_robot_state;

	/// A vector of path segments representing the solution path so far.
	std::vector<std::pair<RobotPath, utilities::GoalEvent>> solution_path_segments;
public:
	const moveit::core::RobotState &getLastRobotState() const;

	const std::vector<std::pair<RobotPath, utilities::GoalEvent>> &getSolutionPathSegments() const;

private:

	/// A vector of discovery statuses.
	std::vector<utilities::DiscoveryStatus> discovery_status;

	/// A reference to the planning scene.
	AppleTreePlanningScene scene;

	/// An optional GoalEvent object representing the next goal event.
	std::optional<utilities::GoalEvent> upcoming_goal_event;

	/// A boolean indicating whether computeNextTrajectory() has been called yet.
	bool first_call = true;

	/**
	 * Internal function to compute to request the planner to replan from the last waypoint
	 * in the current path, assuming we have a valid upcoming_goal_event.
	 *
	 * @return 				An optional PathSegment object representing the new path segment.
	 */
	[[nodiscard]] std::optional<MoveitPathSegment> replanFromEvent();

public:

	/**
	 * @brief Constructor for DynamicGoalVisitationEvaluation.
	 * @param planner A shared pointer to the planner being evaluated.
	 * @param initial_state The initial state of the robot.
	 * @param scene A reference to the AppleTreePlanningScene.
	 * @param discoverability A vector of AppleDiscoverabilityType objects.
	 */
	DynamicGoalVisitationEvaluation(std::shared_ptr<Planner> planner,
									const moveit::core::RobotState &initial_state,
									const AppleTreePlanningScene &scene,
									const std::vector<AppleDiscoverabilityType> &discoverability);

	/**
	 * @brief Computes the next robot trajectory.
	 * @return An optional RobotTrajectory object representing the next trajectory.
	 */
	std::optional<robot_trajectory::RobotTrajectory> computeNextTrajectory();

	/**
	 * @brief Returns the next upcoming recomputation event after the last path emitted from computeNextTrajectory().
	 *
	 * Note: will be std::nullopt if either computeNextTrajectory() has not been called yet,
	 * or if the last call to computeNextTrajectory() returned a path that will not discover
	 * any new apples.
	 *
	 * @return An optional RecomputationEvent object representing the recomputation event.
	 */
	[[nodiscard]] const std::optional<utilities::GoalEvent> &getUpcomingGoalEvent() const;

	/**
	 * @brief Returns the discovery status of each apple.
	 * @return A vector of DiscoveryStatus objects; every index corresponds to an apple, in the same order as the apples in the planning scene.`
	 */
	[[nodiscard]] const std::vector<utilities::DiscoveryStatus> &getDiscoveryStatus() const;

	AppleTreePlanningScene getCurrentCensoredScene();
};


#endif //NEW_PLANNERS_DYNAMICGOALVISITATIONEVALUATION_H
