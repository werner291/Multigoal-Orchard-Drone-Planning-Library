// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 14-2-24.
//

#include "RobotPath.h"

#include <numeric>

mgodpl::RobotState mgodpl::interpolate(const mgodpl::PathPoint &path_point, const mgodpl::RobotPath &robot_path) {
	// Check if the segment index is valid
	assert(path_point.segment_i < robot_path.states.size() - 1 && "Segment index is out of bounds");

	// Get the two states that correspond to the segment
	const RobotState &state1 = robot_path.states[path_point.segment_i];
	const RobotState &state2 = robot_path.states[path_point.segment_i + 1];

	// Interpolate between the two states
	return interpolate(state1, state2, path_point.segment_t);
}

double mgodpl::calculateSegmentLength(const mgodpl::RobotPath &robot_path, const mgodpl::PathPoint &path_point,
									  DistanceFn distanceFunc) {
	const auto &start_state = robot_path.states[path_point.segment_i];
	const auto &end_state = robot_path.states[path_point.segment_i + 1];
	return distanceFunc(start_state, end_state);
}

bool mgodpl::clampPathPoint(const mgodpl::RobotPath &robot_path, mgodpl::PathPoint &path_point) {
	assert(robot_path.states.size() >= 2 && "Robot path must have at least two states to clamp");
	if (path_point.segment_i + 1 >= robot_path.states.size()) {
		path_point.segment_i = robot_path.states.size() - 2;
		path_point.segment_t = 1.0;
		return true;
	}
	return false;
}

bool mgodpl::wrapPathPoint(const mgodpl::RobotPath &robot_path, mgodpl::PathPoint &path_point) {
	if (path_point.segment_i + 1 >= robot_path.states.size()) {
		path_point.segment_i = 0;
		return true;
	}
	return false;
}

void advance_naive(mgodpl::PathPoint &path_point, double advancement, double segment_length) {
	path_point.segment_t += advancement / segment_length;

	if (path_point.segment_t > 1.0) {
		path_point.segment_i++;
		path_point.segment_t = 0.0;
	}
}

bool mgodpl::advancePathPointClamp(const mgodpl::RobotPath &robot_path, mgodpl::PathPoint &path_point,
								   double advancement, DistanceFn distanceFunc) {
	double segment_length = calculateSegmentLength(robot_path, path_point, distanceFunc);
	advance_naive(path_point, advancement, segment_length);
	return clampPathPoint(robot_path, path_point);
}

bool mgodpl::advancePathPointWrap(const mgodpl::RobotPath &robot_path, mgodpl::PathPoint &path_point,
								  double advancement, DistanceFn distanceFunc) {
	double segment_length = calculateSegmentLength(robot_path, path_point, distanceFunc);
	advance_naive(path_point, advancement, segment_length);
	return wrapPathPoint(robot_path, path_point);
}

mgodpl::RobotPath mgodpl::concatenate(const RobotPath &path1, const RobotPath &path2) {
	RobotPath result;
	result.states = path1.states;
	result.states.insert(result.states.end(), path2.states.begin(), path2.states.end());
	return result;
}

mgodpl::RobotPath mgodpl::reverse(const RobotPath &path) {
	RobotPath result;
	result.states = path.states;
	std::reverse(result.states.begin(), result.states.end());
	return result;
}

mgodpl::RobotPath mgodpl::subdivided(const mgodpl::RobotPath &original, size_t num_steps) {

	std::vector<RobotState> new_states;

	for (size_t segment_i = 0; segment_i + 1 < original.states.size(); segment_i++) {
		for (size_t step_i = 0; step_i < num_steps; step_i++) {
			double t = (double) step_i / (double) num_steps;
			new_states.push_back(interpolate(original.states[segment_i], original.states[segment_i + 1], t));
		}
	}

	return {new_states};

}

namespace mgodpl {
	double pathLength(const RobotPath &path, const DistanceFn &distanceFunc) {
		return std::accumulate(path.states.begin(),
							   path.states.end() - 1,
							   0.0,
							   [&distanceFunc](double acc, const RobotState &state) {
								   return acc + distanceFunc(state, *(std::next(&state)));
							   });
	}

	double pathRoughness(const RobotPath &path,
						 const std::function<double(const RobotState &,
													const RobotState &,
													const RobotState &)> &roughnessFunc) {

		double roughness = 0.0;
		for (size_t i = 1; i + 1 < path.states.size(); ++i) {
			roughness += roughnessFunc(path.states[i - 1], path.states[i], path.states[i + 1]);
		}
		return roughness;

	}

}