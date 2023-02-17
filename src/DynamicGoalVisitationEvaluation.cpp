// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/filter.hpp>
#include "DynamicGoalVisitationEvaluation.h"
#include "utilities/moveit.h"

DynamicGoalVisitationEvaluation::DynamicGoalVisitationEvaluation(std::shared_ptr<DynamicMultiGoalPlanner> planner,
																 const moveit::core::RobotState &initial_state,
																 const AppleTreePlanningScene &scene,
																 const std::vector<AppleDiscoverabilityType> &discoverability,
																 const ompl::base::SpaceInformationPtr &si) : planner(
		std::move(planner)), robot_state(initial_state), scene(scene), si(si) {

	ss = std::dynamic_pointer_cast<DroneStateSpace>(si->getStateSpace());

	// Construct goals using the scene and discoverability vector
	goals = ranges::views::ints(0, (int) scene.apples.size()) | ranges::views::filter([&](int goal_id) {
		return discoverability[goal_id] == AppleDiscoverabilityType::GIVEN;
	}) | ranges::views::transform([&](int goal_id) {
		auto goal = std::make_shared<DroneEndEffectorNearTarget>(si, 0.05, scene.apples[goal_id].center);
		return std::static_pointer_cast<ompl::base::Goal>(goal);
	}) | ranges::to_vector;

	// Initialize discovery status vector to "exists but unknown to robot" for all apples
	discovery_status.resize(scene.apples.size(), utilities::DiscoveryStatus::EXISTS_BUT_UNKNOWN_TO_ROBOT);

	// Update discovery status vector for apples that are given
	for (const auto &[goal_id, discoverability_apple]: ranges::views::zip(ranges::views::ints(0,
																							  (int) scene.apples
																									  .size()),
																		  discoverability)) {
		if (discoverability_apple == AppleDiscoverabilityType::GIVEN) {
			discovery_status[goal_id] = utilities::DiscoveryStatus::KNOWN_TO_ROBOT;
		}
	}
}

std::optional<robot_trajectory::RobotTrajectory> DynamicGoalVisitationEvaluation::computeNextTrajectory() {

	// Copy the current robot state to OMPL state space
	ompl::base::ScopedState start(si);
	ss->copyToOMPLState(start.get(), robot_state);

	// Check if either there is an upcoming goal event or this is the first call to this function
	// If not, there is no new information and the planner should not be called; if it wanted to
	// replan, it should have done so when previously called.

	if (upcoming_goal_event || first_call) {
		// Set the first_call flag to false since we just called the function.
		first_call = false;

		// If there is an upcoming goal event, replan
		std::optional<DynamicMultiGoalPlanner::PathSegment> segment;

		// If there is an upcoming goal event, feed the information to the planner and request a replan
		if (upcoming_goal_event) {
			segment = replanFromEvent(start.get());
		} else {
			// If there is no upcoming goal event, plan from scratch
			segment = planner->plan(si, start.get(), goals, scene);
		}

		// If the planner failed to find a solution, or if the solution is empty, return std::nullopt,
		// indicating that the end of the evaluation has been reached.
		if (!segment) {
			return std::nullopt;
		}

		// Convert the result path to a RobotPath
		RobotPath path = omplPathToRobotPath(segment->path_);

		// Check if the robot will discover any apples during the trajectory
		auto event = utilities::find_earliest_discovery_event(path, scene.apples, 1.0);

		if (event) {
			// If the robot will discover an apple, set the upcoming goal event to the discovery event
			upcoming_goal_event = *event;

			// Update the discovery status vector to reflect the discovery
			discovery_status[event->goal_id] = utilities::DiscoveryStatus::KNOWN_TO_ROBOT;

			path.truncateUpTo(event->time);


		} else {
			// If the robot will not discover any apples, set the upcoming goal event
			// to the visitation event, coinciding with the planned end of the trajectory
			upcoming_goal_event = utilities::GoalVisit{(int) segment->to_goal_id_};

			// Update the discovery status vector to reflect the visitation
			discovery_status[segment->to_goal_id_] = utilities::DiscoveryStatus::VISITED;


		}
		// Set the robot state to the end of the trajectory
		robot_state = path.waypoints.back();

		// Return the trajectory
		return robotPathToConstantSpeedRobotTrajectory(path, 1.0);

	} else {

		return std::nullopt;

	}
}

std::optional<DynamicMultiGoalPlanner::PathSegment>
DynamicGoalVisitationEvaluation::replanFromEvent(ompl::base::State *start) {
	std::optional<DynamicMultiGoalPlanner::PathSegment> segment;
	switch (upcoming_goal_event->index()) {
		case 0: {
			// If the upcoming goal event is a visitation event
			const auto &visitation_event = std::get<utilities::GoalVisit>(*upcoming_goal_event);
			segment = planner->replan_after_successful_visit(si, start, goals[visitation_event.goal_id], scene);
			break;
		}
		case 1: {
			// If the upcoming goal event is a discovery event
			const auto &discovery_event = std::get<utilities::GoalSighting>(*upcoming_goal_event);
			segment = planner->replan_after_discovery(si,
													  start,
													  goals[discovery_event.goal_id],
													  discovery_event.time,
													  scene);
			break;
		}
		default:
			throw std::runtime_error("Invalid goal event type");
	}
	return segment;
}

const std::vector<utilities::DiscoveryStatus> &DynamicGoalVisitationEvaluation::getDiscoveryStatus() const {
	return discovery_status;
}

const std::optional<utilities::GoalEvent> &DynamicGoalVisitationEvaluation::getUpcomingGoalEvent() const {
	return upcoming_goal_event;
}
