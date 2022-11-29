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
 *
 * The class works by, for every mesh, sampling a number of points on the surface
 * of the mesh.
 *
 * Then, by repeated calls to snapshot(), the class can be updated with the points
 * that were scanned by the robot's camera. The class will keep track of which
 * of the sampled points have been scanned.
 */
class ScannedSurfaceTracker {

	/// The maximum distance that a point can be from the robot's camera to be considered well-enough scanned.
	const double SCAN_MAX_DISTANCE = 1.0;
	/// The number of points to place on each mesh.
	const size_t POINTS_PER_TARGET = 10;

	/// The biggest target ID; using this, we can make the various vectors the right size.
	size_t max_target = 0;


	/// The sampled points.
	std::vector<ScanTargetPoint> points;

	/// For every sampled point, keep a boolean indicating whether it has been scanned.
	std::vector<bool> scanned_points;

	/// A spatial index that can be used to quickly find what points are visible to the robot's camera.
	ScannablePointsIndex scannablePointsIndex;

	/**
	 * Constructor, with pre-sampled points. Refer to the buildForMeshes() method for a way to
	 * generate these points and automatically wrap them in a ScannedSurfaceTracker.
	 *
	 * @param points 	The sampled points.
	 */
	explicit ScannedSurfaceTracker(const std::vector<ScanTargetPoint> &points);

public:

	/**
	 * Build a ScannedSurfaceTracker for a set of meshes.
	 *
	 * @param meshes 		The meshes to build the ScannedSurfaceTracker for.
	 * @return 				A ScannedSurfaceTracker for the given meshes.
	 */
	ScannedSurfaceTracker buildForMeshes(const std::vector<shape_msgs::msg::Mesh> &meshes);

	/**
	 * Update the tracker with a new snapshot of the robot's camera.
	 *
	 * @param sensor_pose 			The pose of the robot's camera.
	 * @param visible_points 		The target points that the robot's camera can see.
	 */
	void
	snapshot(const Eigen::Isometry3d &sensor_pose, const std::vector<SegmentedPointCloud::TargetPoint> &visible_points);

	/**
	 * Get the (approximate) proportion (0 to 1) of the surface of each mesh that has been scanned.
	 *
	 * The vector index corresponds to the vector of meshes that was passed to buildForMeshes().
	 *
	 * @return The vector of proportions, where the index is the mesh ID.
	 */
	std::vector<double> per_target_scan_portion() const;

};

#endif //NEW_PLANNERS_SCANNEDSURFACETRACKER_H
