// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "DynamicGoalVisitationEvaluation.h"

DynamicGoalVisitationEvaluation::DynamicGoalVisitationEvaluation(std::shared_ptr<DynamicMultiGoalPlanner> planner,
																 const moveit::core::RobotState &initial_state,
																 const AppleTreePlanningScene &scene,
																 const std::vector<AppleDiscoverabilityType> &discoverability,
																 const ompl::base::SpaceInformationPtr &si) : planner(
		std::move(planner)), robot_state(initial_state), scene(scene), si(si) {

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

	// Initialize a non-terminating condition for the planner
	auto ptc = ompl::base::plannerNonTerminatingCondition();

	// Plan or replan depending on whether a recomputation event occurred in the previous iteration
	auto result = re ? planner->replan(si, start.get(), re->goal_changes, scene, ptc) : planner->plan(si,
																									  start.get(),
																									  goals,
																									  scene,
																									  ptc);

	// Convert the result path to a RobotPath
	RobotPath path = omplPathToRobotPath(result.combined());

	// Convert the RobotPath to a RobotTrajectory with constant speed
	robot_trajectory::RobotTrajectory traj = robotPathToConstantSpeedRobotTrajectory(path, 1.0);

	// Find goal events along the trajectory
	auto events = utilities::goal_events(traj, scene.apples, 0.1, 0.5);

	// Find the next recomputation event using the goal events and discovery status
	re = find_recomputation_event(events, goals, discovery_status);
}
