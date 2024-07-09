// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/19/24.
//

#include "state_tools.h"

#include "RandomNumberGenerator.h"
#include "spherical_geometry.h"

mgodpl::RobotState mgodpl::fromEndEffectorAndVector(const mgodpl::robot_model::RobotModel &robot,
													const mgodpl::math::Vec3d &endEffectorPoint,
													const mgodpl::math::Vec3d &vector) {

	// Generate a state.
	std::vector<double> arm_angles { -spherical_geometry::latitude(vector) };

	math::Transformd flying_base_tf {
			.translation = math::Vec3d(0.0, 0.0, 0.0),
			.orientation = math::Quaterniond::fromAxisAngle(math::Vec3d::UnitZ(), spherical_geometry::longitude(vector) + M_PI/2.0)
	};

	const auto& fk = mgodpl::robot_model::forwardKinematics(robot, arm_angles, robot.findLinkByName("flying_base"), flying_base_tf);

	math::Vec3d end_effector_position = fk.forLink(robot.findLinkByName("end_effector")).translation;

	flying_base_tf.translation = flying_base_tf.translation + endEffectorPoint - end_effector_position;

	return {
			.base_tf = flying_base_tf,
			.joint_values = arm_angles,
	};
}

mgodpl::RobotState mgodpl::generateUniformRandomState(const robot_model::RobotModel &robot,
                                                      random_numbers::RandomNumberGenerator &rng,
                                                      const double h_range,
                                                      const double v_range,
                                                      const double joint_angle_range) {
	RobotState state{
		.base_tf = {
			.translation = {
				rng.uniformReal(-h_range, h_range),
				rng.uniformReal(-h_range, h_range),
				rng.uniformReal(0, v_range)
			},
			.orientation = math::Quaterniond::fromAxisAngle(math::Vec3d::UnitZ(), rng.uniformReal(0, 2 * M_PI))
		},
		.joint_values = {}
	};

	state.joint_values.reserve(robot.getJoints().size());
	for (size_t i = 0; i < robot.getJoints().size(); ++i) {
		state.joint_values.push_back(rng.uniformReal(-joint_angle_range, joint_angle_range));
	}

	return state;
}
