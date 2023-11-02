// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/2/23.
//

#include <algorithm>
#include <moveit/robot_state/robot_state.h>
#include "BlindlyMoveToNextFruit.h"
#include "JointSpacePoint.h"

namespace mgodpl::planning {

	using namespace moveit_facade;

	std::optional<JointSpacePoint> BlindlyMoveToNextFruit::nextMovement(const mgodpl::planning::RobotAlgorithm::ExternalStateUpdate &state) {

		fruit_to_visit.insert(fruit_to_visit.end(),
							  state.newly_detected_fruits.begin(),
							  state.newly_detected_fruits.end());

		std::cout << "Fruit to visit: " << fruit_to_visit.size() << std::endl;

		// If no more fruit, return nothing.
		if (fruit_to_visit.empty())
			return std::nullopt;

		// Get the end-effector position.
		math::Vec3d ee_pos = computeEndEffectorPosition(*robot_model, state.current_state);

		// Find the closest fruit.
		auto closest_fruit = std::min_element(fruit_to_visit.begin(),
											  fruit_to_visit.end(),
											  [&](const math::Vec3d &a, const math::Vec3d &b) {
												  return (a - ee_pos).norm() < (b - ee_pos).norm();
											  });

		JointSpacePoint next_state = state.current_state;

		// Move the end-effector to the closest fruit.
		experiment_state_tools::moveEndEffectorToPoint(*robot_model, next_state, *closest_fruit);

		// Remove the fruit from the list of fruit to visit.
		fruit_to_visit.erase(closest_fruit);

		return next_state;

	}
}