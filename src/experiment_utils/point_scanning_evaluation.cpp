// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/11/24.
//

#include "point_scanning_evaluation.h"

using namespace mgodpl;

mgodpl::EvaluationTrace mgodpl::eval_static_path(const mgodpl::RobotPath &path,
												 double interpolation_speed,
												 const std::vector<std::vector<SurfacePoint>> &all_scannable_points,
												 const SensorScalarParameters &sensor_params,
												 const std::shared_ptr<const MeshOcclusionModel> &mesh_occlusion_model) {

	// Define the current position on the path
	PathPoint path_point = {0, 0.0};

	std::vector<std::vector<bool>> ever_seen = init_seen_status(all_scannable_points);

	// Initialize an empty JSON object to store the statistics
	EvaluationTrace stats;

	// Put the robot at the current point (the start of the path)
	RobotState last_state = interpolate(path_point, path);

	// Loop until the path is completed; function will return true when the path is completed.
	while (!advancePathPointClamp(path, path_point, interpolation_speed, equal_weights_max_distance)) {

		// Interpolate the robot's state
		auto interpolated_state = interpolate(path_point, path);

		// Record inter-joint distances
		JointDistances jd = calculateJointDistances(last_state, interpolated_state);

		// Get the position/forward of the robot's end effector
		const auto &end_effector_position = interpolated_state.base_tf.translation;
		math::Vec3d end_effector_forward = interpolated_state.base_tf.orientation.rotate(math::Vec3d(0, 1, 0));

		// Update the visibility of the scannable points
		update_seen(
				sensor_params,
				mesh_occlusion_model,
				end_effector_position,
				end_effector_forward, all_scannable_points,
				ever_seen);

		// Count the number of points seen for each fruit so far.
		std::vector<size_t> seen_counts;
		for (size_t fruit_i = 0; fruit_i < all_scannable_points.size(); ++fruit_i) {
			seen_counts.push_back(std::count(ever_seen[fruit_i].begin(), ever_seen[fruit_i].end(), true));
		}

		// Add the current frame to the statistics
		stats.frames.push_back({jd, seen_counts});
	}

	// After we're done, return the full trace.
	return stats;
}

Json::Value mgodpl::toJson(const mgodpl::EvaluationTrace &trace) {
	Json::Value json;
	json["frames"] = Json::arrayValue;
	for (const auto &frame: trace.frames) {
		Json::Value frame_json;
		frame_json["joint_distances"] = toJson(frame.joint_distances);
		frame_json["pts_seen"] = Json::arrayValue;
		for (size_t pts_seen: frame.pts_seen) {
			frame_json["pts_seen"].append(pts_seen);
		}
		json["frames"].append(frame_json);
	}
	return json;
}

std::vector<std::vector<bool>> mgodpl::init_seen_status(const std::vector<std::vector<SurfacePoint>> &all_scannable_points) {
	std::vector<std::vector<bool>> ever_seen;
	ever_seen.reserve(all_scannable_points.size());
	for (const auto &scannable_points: all_scannable_points) {
		ever_seen.emplace_back(scannable_points.size(), false);
	}
	return ever_seen;
}

void mgodpl::update_seen(const SensorScalarParameters &sensor_params,
				 const std::shared_ptr<const MeshOcclusionModel> &mesh_occlusion_model,
				 const math::Vec3d &eye_position,
				 const math::Vec3d &eye_forward,
				 const std::vector<std::vector<SurfacePoint>> &all_scannable_points,
				 std::vector<std::vector<bool>> &ever_seen) {
	for (size_t fruit_i = 0; fruit_i < all_scannable_points.size(); ++fruit_i) {
		for (size_t i = 0; i < all_scannable_points[fruit_i].size(); ++i) {
			if (!ever_seen[fruit_i][i] &&
				is_visible(all_scannable_points[fruit_i][i],
						   eye_position,
						   eye_forward,
						   sensor_params.maxViewDistance,
						   sensor_params.minViewDistance,
						   sensor_params.maxScanAngle,
						   sensor_params.fieldOfViewAngle,
						   *mesh_occlusion_model)) {
				ever_seen[fruit_i][i] = true;
			}
		}
	}
}