// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "ScannedSurfaceTracker.h"

ScannedSurfaceTracker ScannedSurfaceTracker::buildForMeshes(const std::vector<shape_msgs::msg::Mesh> &meshes) {
	return ScannedSurfaceTracker(buildScanTargetPoints(meshes, POINTS_PER_TARGET));

}

ScannedSurfaceTracker::ScannedSurfaceTracker(const std::vector<ScanTargetPoint> &points)
		: points(points), scanned_points(points.size(), false), scannablePointsIndex(points) {

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
	std::vector<double> per_fruit_scan_proportion(max_target + 1, 0.0);

	for (size_t scan_point_id = 0; scan_point_id < scanned_points.size(); scan_point_id++) {
		per_fruit_scan_proportion[points[scan_point_id].apple_id] +=
				scanned_points[scan_point_id] / (double) POINTS_PER_TARGET;
	}

	return per_fruit_scan_proportion;
}
