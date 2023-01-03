// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-12-22.
//

#ifndef NEW_PLANNERS_OCCUPANCYMAP_H
#define NEW_PLANNERS_OCCUPANCYMAP_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "../exploration/SegmentedPointCloud.h"
#include "VisibilityBoundary.h"

/**
* A struct for determining if a cell should be split based on the distance between the center
* of the cell and the surface point of the region boundary.
*/
template<typename LeafCell>
struct SplitIfBoundaryMaybeInsideCell {
	const Eigen::Vector3d &eye_center;
	const RegionDefinitionFn &region_fn;

	/**
	 * The call operator for determining if a cell should be split.
	 *
	 * @param box The bounding box of the cell.
	 * @param leaf_cell The cell to potentially split.
	 * @return True if the cell should be split, false otherwise.
	 */
	bool operator()(const Eigen::AlignedBox3d &box, LeafCell &leaf_cell) const {
		// Split a cell if the boundary of the region may lie inside of it.
		BoundarySample boundary_sample = region_fn(box.center());

		return (box.center() - boundary_sample.surface_point).norm() <= box.sizes()[0] * sqrt(3) / 2.0;
	}
};

class OccupancyMap {

public:
	struct OccludingPoint {
		Eigen::Vector3d point;
		bool hard;
	};

	/**
	 * @fn virtual void incorporate(const Eigen::Vector3d &eye_center, const SegmentedPointCloud &pointCloud)
	 *
	 * Incorporate a segmented point cloud into the occupancy map.
	 *
	 * @param eye_transform 		The center of the robot's sensor, used as a reference point for incorporating the point cloud.
	 * @param pointCloud 		The segmented point cloud to be incorporated into the occupancy map.
	 */
	virtual void incorporate(const std::vector<OccludingPoint> &occluding_points,
							 const Eigen::Isometry3d &eye_transform,
							 double fovX,
							 double fovY) = 0;

	/// Destructor.
	virtual ~OccupancyMap() = default;

};

#endif //NEW_PLANNERS_OCCUPANCYMAP_H
