// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/11/24.
//

#include "joint_distances.h"

mgodpl::JointDistances mgodpl::calculateJointDistances(const RobotState &state1, const RobotState &state2) {
	JointDistances distances;

	distances.translation_distance = (state2.base_tf.translation - state1.base_tf.translation).norm();
	distances.rotation_distance = angular_distance(state2.base_tf.orientation, state1.base_tf.orientation);

	for (size_t i = 0; i < state1.joint_values.size(); ++i) {
		distances.joint_distances.push_back(std::abs(state2.joint_values[i] - state1.joint_values[i]));
	}

	return distances;
}

Json::Value mgodpl::toJson(const mgodpl::JointDistances &distances) {
	Json::Value json;
	json["translation_distance"] = distances.translation_distance;
	json["rotation_distance"] = distances.rotation_distance;
	json["joint_distances"] = Json::arrayValue;
	for (const auto &joint_distance: distances.joint_distances) {
		json["joint_distances"].append(joint_distance);
	}
	return json;
}
