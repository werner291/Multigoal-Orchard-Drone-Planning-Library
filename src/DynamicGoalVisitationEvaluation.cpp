// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/filter.hpp>
#include "DynamicGoalVisitationEvaluation.h"
#include "utilities/moveit.h"

utilities::DiscoveryStatus initial_discovery_status(const AppleDiscoverabilityType &discoverabilityType) {
	switch (discoverabilityType) {
		case AppleDiscoverabilityType::GIVEN:
			return utilities::DiscoveryStatus::KNOWN_TO_ROBOT;
		case AppleDiscoverabilityType::DISCOVERABLE:
			return utilities::DiscoveryStatus::EXISTS_BUT_UNKNOWN_TO_ROBOT;
	}

	throw std::runtime_error("Invalid discoverability type");
}

DynamicGoalVisitationEvaluation::DynamicGoalVisitationEvaluation(std::shared_ptr<Planner> planner,
																 const moveit::core::RobotState &initial_state,
																 const AppleTreePlanningScene &scene,
																 const std::vector<AppleDiscoverabilityType> &discoverability)
		: planner(std::move(planner)), last_robot_state(initial_state), scene(scene) {
	// Update discovery status vector for apples that are given
	discovery_status = discoverability | ranges::views::transform(initial_discovery_status) | ranges::to_vector;
}

std::optional<robot_trajectory::RobotTrajectory> DynamicGoalVisitationEvaluation::computeNextTrajectory() {

	// Check if either there is an upcoming goal event or this is the first call to this function
	// If not, there is no new information and the planner should not be called; if it wanted to
	// re-plan, it should have done so when previously called.
	if (upcoming_goal_event || first_call) {

		// Set the first_call flag to false since we just called the function.
		first_call = false;

		// If there is an upcoming goal event, replan
		std::optional<MoveitPathSegment> segment;

		// If there is an upcoming goal event, feed the information to the planner and request a replan
		if (upcoming_goal_event) {
			segment = replanFromEvent();
		} else {
			// If there is no upcoming goal event, plan from scratch
			segment = planner->plan(last_robot_state, scene);
		}

		// If the planner failed to find a solution, or if the solution is empty, return std::nullopt,
		// indicating that the end of the evaluation has been reached.
		if (!segment) {
			return std::nullopt;
		}

		// Validate to ensure the segment starts at the current robot state
		if (segment->path.waypoints[0].distance(last_robot_state) > 1e-6) {
			std::cout << "Off by " << segment->path.waypoints[0].distance(last_robot_state) << " units" << std::endl;
			throw std::runtime_error("Planner returned a path that does not start at the current robot state");
		}

		// Check if the robot will discover any apples during the trajectory
		auto event = utilities::find_earliest_discovery_event(segment->path, scene.apples, 1.0, discovery_status);

		if (event) {

			// If the robot will discover an apple, set the upcoming goal event to the discovery event
			upcoming_goal_event = *event;

			// Update the discovery status vector to reflect the discovery
			discovery_status[event->goal_id] = utilities::DiscoveryStatus::KNOWN_TO_ROBOT;

			segment->path.truncateUpTo(event->time);

		} else {

			// Among the apples, find all that are in range of the robot's end-effector
			Eigen::Vector3d ee_pos = segment->path
					.waypoints
					.back()
					.getGlobalLinkTransform("end_effector")
					.translation();

			// TODO The ID tracking is a bit weird here, need to fix.
			for (size_t apple_i = 0; apple_i < scene.apples.size(); apple_i++) {
				if ((scene.apples[apple_i].center - ee_pos).norm() < 0.05) {
					// If the apple is in range, update the discovery status vector to reflect the visitation
					discovery_status[apple_i] = utilities::DiscoveryStatus::VISITED;

					// If the robot will not discover any apples, set the upcoming goal event
					// to the visitation event, coinciding with the planned end of the trajectory
					upcoming_goal_event = utilities::GoalVisit{(int) segment->goal_id};
				}
			}

		}

		// Set the robot state to the end of the trajectory
		last_robot_state = segment->path.waypoints.back();

		this->solution_path_segments.emplace_back(segment->path, *upcoming_goal_event);

		// Return the trajectory
		return robotPathToConstantSpeedRobotTrajectory(segment->path, 1.0);

	} else {

		return std::nullopt;

	}
}

std::optional<MoveitPathSegment> DynamicGoalVisitationEvaluation::replanFromEvent() {
	assert(upcoming_goal_event.has_value());

	std::optional<MoveitPathSegment> segment;
	switch (upcoming_goal_event->index()) {
		case 0: {
			// If the upcoming goal event is a visitation event
			const auto &visitation_event = std::get<utilities::GoalVisit>(*upcoming_goal_event);
			segment = planner->replan_after_successful_visit(last_robot_state, scene);
			break;
		}
		case 1: {
			// If the upcoming goal event is a discovery event
			const auto &discovery_event = std::get<utilities::GoalSighting>(*upcoming_goal_event);
			segment = planner->replan_after_discovery(last_robot_state,
													  scene.apples[discovery_event.goal_id],
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

const moveit::core::RobotState &DynamicGoalVisitationEvaluation::getLastRobotState() const {
	return last_robot_state;
}

const std::vector<std::pair<RobotPath, utilities::GoalEvent>> &
DynamicGoalVisitationEvaluation::getSolutionPathSegments() const {
	return solution_path_segments;
}
