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
#include "ompl_custom.h"
#include "utilities/experiment_utils.h"

/**
 * @brief A class representing an evaluation framework for a given planner.
 *
 * The DynamicGoalVisitationEvaluation class encapsulates the evaluation process for a given planner,
 * allowing other code to focus on gathering statistics and/or visualization as needed.
 */
class DynamicGoalVisitationEvaluation {

	std::shared_ptr<DynamicMultiGoalPlanner> planner; ///< A pointer to the planner being evaluated.

	moveit::core::RobotState robot_state; ///< The initial state of the robot.
	std::optional<utilities::RecomputationEvent> re; ///< An optional recomputation event.

	std::vector<utilities::DiscoveryStatus> discovery_status;
public:
	const std::vector<utilities::DiscoveryStatus> &getDiscoveryStatus() const;

private:
	///< A vector of discovery statuses.

	ompl::base::SpaceInformationPtr si; ///< A pointer to the space information object.
	std::shared_ptr<DroneStateSpace> ss; ///< A pointer to the drone state space.

	std::vector<ompl::base::GoalPtr> goals; ///< A vector of goals.
	const AppleTreePlanningScene &scene; ///< A reference to the planning scene.

public:

	/**
	 * @brief Constructor for DynamicGoalVisitationEvaluation.
	 * @param planner A shared pointer to the planner being evaluated.
	 * @param initial_state The initial state of the robot.
	 * @param scene A reference to the AppleTreePlanningScene.
	 * @param discoverability A vector of AppleDiscoverabilityType objects.
	 * @param si A pointer to the SpaceInformation object.
	 */
	DynamicGoalVisitationEvaluation(std::shared_ptr<DynamicMultiGoalPlanner> planner,
									const moveit::core::RobotState &initial_state,
									const AppleTreePlanningScene &scene,
									const std::vector<AppleDiscoverabilityType> &discoverability,
									const ompl::base::SpaceInformationPtr &si);

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
	[[nodiscard]] const std::optional<utilities::RecomputationEvent> &getRe() const {
		return re;
	}

};


#endif //NEW_PLANNERS_DYNAMICGOALVISITATIONEVALUATION_H
