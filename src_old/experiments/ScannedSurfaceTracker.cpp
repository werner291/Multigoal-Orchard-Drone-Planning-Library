// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "ScannedSurfaceTracker.h"

ScannedSurfaceTracker ScannedSurfaceTracker::buildForMeshes(const std::vector<shape_msgs::msg::Mesh> &meshes) {
	// Sample the points and call the constructor.
	// (not doing this in the constructor because it would cause a RAII violation without relying on mutations)
	return ScannedSurfaceTracker(buildScanTargetPoints(meshes, POINTS_PER_TARGET));
}

ScannedSurfaceTracker::ScannedSurfaceTracker(const std::vector<ScanTargetPoint> &points)
		: points(points), scanned_points(points.size(), false), scannablePointsIndex(points) {

	// Find the maximum target ID, which is expected to be at most in the several hundreds.
	for (const auto &point: points) {
		if (point.apple_id > max_target) {
			max_target = point.apple_id;
		}
	}

}

void ScannedSurfaceTracker::snapshot(const Eigen::Isometry3d &sensor_pose,
									 const std::vector<SegmentedPointCloud::TargetPoint> &visible_points) {

	// Find which of the scannable points are visible to the robot's camera.
	auto new_scanned_points = scannablePointsIndex.findScannedPoints(sensor_pose.translation(),
																	 sensor_pose.rotation() * Eigen::Vector3d(0, 1, 0),
																	 M_PI / 4.0,
																	 SCAN_MAX_DISTANCE,
																	 visible_points);

	for (auto i: new_scanned_points) {
		if (!scanned_points[i]) {
			scanned_points[i] = true;
		}
	}

}

std::vector<double> ScannedSurfaceTracker::per_target_scan_portion() const {

	// Initialize to 0 for all target meshes.
	std::vector<double> per_fruit_scan_proportion(max_target + 1, 0.0);

	// Iterate over all scan points.
	for (size_t scan_point_id = 0; scan_point_id < scanned_points.size(); scan_point_id++) {
		// If the point has been scanned, add to the proportion of the target mesh that has been scanned.
		per_fruit_scan_proportion[points[scan_point_id].apple_id] +=
				scanned_points[scan_point_id] / (double) POINTS_PER_TARGET;
	}

	// Return the proportion of each mesh that has been scanned.
	return per_fruit_scan_proportion;
}
