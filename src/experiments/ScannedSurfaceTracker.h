// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_SCANNEDSURFACETRACKER_H
#define NEW_PLANNERS_SCANNEDSURFACETRACKER_H

#include <cstddef>
#include <vector>
#include <Eigen/Geometry>
#include "../exploration/scan_points.h"
#include "../ScannablePointsIndex.h"

/**
 * Given successive sensor poses and sets of scanned points, estimate
 * the percentage of the surface of each mesh that has been scanned.
 */
class ScannedSurfaceTracker {

	const double SCAN_MAX_DISTANCE = 1.0;
	const size_t POINTS_PER_TARGET = 10;
	size_t max_target = 0;

	std::vector<bool> scanned_points;

	std::vector<ScanTargetPoint> points;

	// Build a spatial index that can be used to quickly find what points are visible to the robot's camera.
	ScannablePointsIndex scannablePointsIndex;

	explicit ScannedSurfaceTracker(const std::vector<ScanTargetPoint> &points);

public:
	ScannedSurfaceTracker buildForMeshes(const std::vector<shape_msgs::msg::Mesh> &meshes);

	void
	snapshot(const Eigen::Isometry3d &sensor_pose, const std::vector<SegmentedPointCloud::TargetPoint> &visible_points);

	std::vector<double> per_target_scan_portion() const;

};

#endif //NEW_PLANNERS_SCANNEDSURFACETRACKER_H
