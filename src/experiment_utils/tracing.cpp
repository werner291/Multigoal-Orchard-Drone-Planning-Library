// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 4/8/24.
//

#include "tracing.h"

std::vector<mgodpl::math::Vec3d> mgodpl::link_trace(const mgodpl::robot_model::RobotModel &robot,
											const mgodpl::RobotPath &final_path,
											const mgodpl::robot_model::RobotModel::LinkId link) {
	// Initialize a vector to store the positions of the link
	std::vector<math::Vec3d> end_effector_positions;

	// Iterate over all states in the path
	for (size_t i = 0; i < final_path.states.size(); ++i) {
		// Compute the forward kinematics for the current state
		const auto fk = forwardKinematics(robot, final_path.states[i].joint_values,
										  robot.findLinkByName("flying_base"), final_path.states[i].base_tf);
		// Add the position of the link to the vector
		end_effector_positions.push_back(fk.forLink(link).translation);
	}

	// Return the vector of positions
	return end_effector_positions;
}
