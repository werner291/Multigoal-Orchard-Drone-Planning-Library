// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/filter.hpp>
#include <utility>
#include "DynamicGoalVisitationEvaluation.h"
#include "utilities/moveit.h"

utilities::DiscoveryStatus initial_discovery_status(const AppleDiscoverabilityType &discoverabilityType) {
	switch (discoverabilityType) {
		case AppleDiscoverabilityType::GIVEN:
			return utilities::DiscoveryStatus::KNOWN_TO_ROBOT;
		case AppleDiscoverabilityType::DISCOVERABLE:
			return utilities::DiscoveryStatus::EXISTS_BUT_UNKNOWN_TO_ROBOT;
		case FALSE:
			return utilities::DiscoveryStatus::ROBOT_THINKS_EXISTS_BUT_DOESNT;
			break;
	}

	throw std::runtime_error("Invalid discoverability type");
}

DynamicGoalVisitationEvaluation::DynamicGoalVisitationEvaluation(std::shared_ptr<Planner> planner,
																 const moveit::core::RobotState &initial_state,
																 const AppleTreePlanningScene &scene,
																 const std::vector<AppleDiscoverabilityType> &discoverability,
																 CanSeeAppleFn canSeeApple)
		: planner(std::move(planner)), last_robot_state(initial_state), scene(scene), can_see_apple(std::move(canSeeApple)) {

	// Set the initial discovery status based on the discoverability vector.
	discovery_status = discoverability | ranges::views::transform(initial_discovery_status) | ranges::to_vector;

	// Additionally, for every apple that the robot can see from its initial position, set the discovery status to KNOWN_TO_ROBOT
	// if the apple is discoverable but not yet known according to the discoverability specification.
	for (auto [i, apple] : scene.apples | ranges::views::enumerate) {
		if (can_see_apple(last_robot_state, apple) && discoverability[i] == AppleDiscoverabilityType::DISCOVERABLE) {
			discovery_status[i] = utilities::DiscoveryStatus::KNOWN_TO_ROBOT;
		}
	}

	assert(scene.apples.size() == discoverability.size());

}

std::optional<DynamicGoalVisitationEvaluation::SolutionPathSegment> DynamicGoalVisitationEvaluation::computeNextTrajectory() {

	// Check if either there is an upcoming goal event or this is the first call to this function
	// If not, there is no new information and the planner should not be called; if it wanted to
	// re-plan, it should have done so when previously called.
	if (upcoming_goal_event || first_call) {

		// Set the first_call flag to false since we just called the function.
		first_call = false;

		// If there is an upcoming goal event, replan
		std::optional<RobotPath> segment;

		auto start_time = std::chrono::high_resolution_clock::now();

		// If there is an upcoming goal event, feed the information to the planner and request a replan
		if (upcoming_goal_event) {
			segment = replanFromEvent();
		} else {

			// If there is no upcoming goal event, plan from scratch.

			AppleTreePlanningScene censored_scene = getCurrentCensoredScene();

			segment = planner->plan(last_robot_state, censored_scene);
		}

		auto end_time = std::chrono::high_resolution_clock::now();

		// If the planner failed to find a solution, or if the solution is empty, return std::nullopt,
		// indicating that the end of the evaluation has been reached.
		if (!segment) {
			return std::nullopt;
		}

		// Validate to ensure the segment starts at the current robot state
		if (segment->waypoints[0].distance(last_robot_state) > 1e-6) {
			std::cout << "Off by " << segment->waypoints[0].distance(last_robot_state) << " units" << std::endl;
			throw std::runtime_error("Planner returned a path that does not start at the current robot state");
		}

		// Check if the robot will discover any apples during the trajectory
		auto event = utilities::find_earliest_discovery_event(*segment, scene.apples, can_see_apple, discovery_status);

		// Check what the original goal of the path segment was by finding the apple closest to the end of the path segment.
		std::optional<utilities::GoalId> apple_id {};
		double min_dist = 0.05;

		for (auto [i, apple] : scene.apples | ranges::views::enumerate) {
			double dist = (segment->waypoints.back().getGlobalLinkTransform("end_effector").translation() - apple.center).norm();
			if (dist < min_dist) {
				min_dist = dist;
				apple_id = (utilities::GoalId) i;
			}
		}

		if (event) {

			// If the robot will discover an apple, set the upcoming goal event to the discovery event
			upcoming_goal_event = *event;

			// Update the discovery status vector to reflect the discovery
			if (discovery_status[event->goal_id] == utilities::DiscoveryStatus::EXISTS_BUT_UNKNOWN_TO_ROBOT) {
				discovery_status[event->goal_id] = utilities::DiscoveryStatus::KNOWN_TO_ROBOT;
			} else {
				assert(discovery_status[event->goal_id] == utilities::DiscoveryStatus::ROBOT_THINKS_EXISTS_BUT_DOESNT);
				discovery_status[event->goal_id] = utilities::DiscoveryStatus::REMOVED;
			}

			segment->truncateUpTo(event->time);

		} else {

			// Check if the end-effector is at any of the goals.
			Eigen::Vector3d end_effector_pos = segment->waypoints
					.back()
					.getGlobalLinkTransform("end_effector")
					.translation();

			std::vector<utilities::GoalId> goals;

			for (size_t apple_i = 0; apple_i < scene.apples.size(); apple_i++) {


				// FIXME: False-positives are flagged as successes soetimes.
				bool is_actual_target = discovery_status[apple_i] == utilities::DiscoveryStatus::KNOWN_TO_ROBOT ||
										discovery_status[apple_i] == utilities::DiscoveryStatus::EXISTS_BUT_UNKNOWN_TO_ROBOT ||
										discovery_status[apple_i] == utilities::DiscoveryStatus::VISITED;

				if (is_actual_target && (end_effector_pos - scene.apples[apple_i].center).norm() < 0.05) {
					discovery_status[apple_i] = utilities::DiscoveryStatus::VISITED;
					goals.emplace_back(apple_i);
				}
			}

			upcoming_goal_event = utilities::PathEnd{goals};

		}

		// Set the robot state to the end of the trajectory
		last_robot_state = segment->waypoints.back();

		SolutionPathSegment segment_to_add {
			*segment,
			apple_id,
			*upcoming_goal_event,
			end_time - start_time};

		this->solution_path_segments.push_back(segment_to_add);

		// Return the trajectory
		return std::make_optional(segment_to_add);

	} else {

		return std::nullopt;

	}
}

AppleTreePlanningScene DynamicGoalVisitationEvaluation::getCurrentCensoredScene() {
	AppleTreePlanningScene censored_scene = scene;

	assert(scene.apples.size() == discovery_status.size());

	censored_scene.apples =
			censored_scene.apples | ranges::views::enumerate | ranges::views::filter([&](const auto &apple) {
				return discovery_status[apple.first] == utilities::DiscoveryStatus::KNOWN_TO_ROBOT ||
					   discovery_status[apple.first] == utilities::DiscoveryStatus::ROBOT_THINKS_EXISTS_BUT_DOESNT;
			}) | ranges::views::transform([](const auto &apple) {
				return apple.second;
			})
			| ranges::to_vector;

	return censored_scene;
}

std::optional<RobotPath> DynamicGoalVisitationEvaluation::replanFromEvent() {
	assert(upcoming_goal_event.has_value());

	std::optional<RobotPath> segment;
	switch (upcoming_goal_event->index()) {
		case 0: {
			// If the upcoming goal event is a visitation event
			const auto &visitation_event = std::get<utilities::PathEnd>(*upcoming_goal_event);
			segment = planner->replan_after_successful_visit(last_robot_state, scene);
			break;
		}
		case 1: {
			// If the upcoming goal event is a discovery event
			const auto &discovery_event = std::get<utilities::GoalSighting>(*upcoming_goal_event);

			assert(discovery_status[discovery_event.goal_id] == utilities::DiscoveryStatus::KNOWN_TO_ROBOT ||
				   discovery_status[discovery_event.goal_id] == utilities::DiscoveryStatus::REMOVED);

			if (discovery_status[discovery_event.goal_id] == utilities::DiscoveryStatus::KNOWN_TO_ROBOT) {
				segment = planner->replan_after_discovery(last_robot_state,
														  scene.apples[discovery_event.goal_id],
														  discovery_event.time,
														  scene);
			} else {
				segment = planner->replan_after_removal(last_robot_state,
														scene.apples[discovery_event.goal_id],
														discovery_event.time,
														scene);
			}

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

const std::vector<DynamicGoalVisitationEvaluation::SolutionPathSegment> &
DynamicGoalVisitationEvaluation::getSolutionPathSegments() const {
	return solution_path_segments;
}

const CanSeeAppleFn &DynamicGoalVisitationEvaluation::getCanSeeApple() const {
	return can_see_apple;
}

void DynamicGoalVisitationEvaluation::runTillCompletion() {

	while (computeNextTrajectory().has_value()) {
		// Do nothing, the condition is the loop body
	}

}
