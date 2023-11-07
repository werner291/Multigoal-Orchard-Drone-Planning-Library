// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/2/23.
//

#include "JointSpacePoint.h"
#include "moveit_forward_declarations.h"

#include <moveit/robot_state/robot_state.h>

namespace mgodpl::moveit_facade {

	void JointSpacePoint::to_moveit(moveit::core::RobotState &state, bool compute_fk) const {
		state.setVariablePositions(joint_values);
		if (compute_fk)
			state.update(true);
	}

	JointSpacePoint JointSpacePoint::from_moveit(const moveit::core::RobotState &state) {

		std::vector<double> joint_values;

		int n = state.getVariableCount();

		joint_values.reserve(n);

		for (int i = 0; i < n; ++i) {
			joint_values.push_back(state.getVariablePosition(i));
		}

		return {joint_values};

	}

	bool JointSpacePoint::significantly_different_from(const JointSpacePoint &other, double threshold) const {
		for (int i = 0; i < joint_values.size(); ++i) {
			if (std::abs(joint_values[i] - other.joint_values[i]) > threshold) {
				return true;
			}
		}
		return false;
	}

	math::Vec3d computeEndEffectorPosition(const moveit::core::RobotModel &robot, const JointSpacePoint &state) {
		return computeLinkEffectorPosition(robot, state, "end_effector");
	}

	double
	moveit_joint_distance(const moveit::core::RobotModel &robot, const JointSpacePoint &a, const JointSpacePoint &b) {

		return robot.distance(a.joint_values.data(), b.joint_values.data());

	}

	JointSpacePoint
	interpolate(const moveit::core::RobotModel &robot, const JointSpacePoint &a, const JointSpacePoint &b, double t) {

		JointSpacePoint result = a;

		robot.interpolate(a.joint_values.data(), b.joint_values.data(), t, result.joint_values.data());

		return result;

	}

	math::Vec3d computeLinkEffectorPosition(const moveit::core::RobotModel &robot,
											const JointSpacePoint &state,
											const std::string &link_name) {

		moveit::core::RobotModelConstPtr fake_shared_ptr(&robot, [](const moveit::core::RobotModel *) {});
		moveit::core::RobotState moveit_state(fake_shared_ptr);
		state.to_moveit(moveit_state, true);
		moveit_state.update(true);

		return math::Vec3d(moveit_state.getGlobalLinkTransform(link_name).translation().data());

	}
}