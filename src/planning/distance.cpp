// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#include "distance.h"

double mgodpl::base_translation_distance(const RobotState &a, const RobotState &b) {
	return (a.base_tf.translation - b.base_tf.translation).norm();
}

double mgodpl::base_distance(const RobotState &a, const RobotState &b, const double rotation_weight) {
	return base_translation_distance(a, b) + rotation_weight * angular_distance(
			a.base_tf.orientation,
			b.base_tf.orientation);
}

double mgodpl::equal_weights_distance(const RobotState &a, const RobotState &b) {
	double d = (a.base_tf.translation - b.base_tf.translation).norm();
	d += angular_distance(a.base_tf.orientation, b.base_tf.orientation);
	for (size_t i = 0; i < a.joint_values.size(); ++i) {
		d += std::abs(a.joint_values[i] - b.joint_values[i]);
	}
	assert(std::isfinite(d));
	return d;
}

double mgodpl::equal_weights_max_distance(const RobotState &a, const RobotState &b) {
	double d = (a.base_tf.translation - b.base_tf.translation).norm();
	d = std::max(d, angular_distance(a.base_tf.orientation, b.base_tf.orientation));
	for (size_t i = 0; i < a.joint_values.size(); ++i) {
		d = std::max(d, std::abs(a.joint_values[i] - b.joint_values[i]));
	}
	assert(std::isfinite(d));
	return d;
}