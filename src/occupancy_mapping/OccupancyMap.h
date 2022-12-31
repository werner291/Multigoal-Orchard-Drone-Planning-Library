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

	/**
	 * An enum representing the type of a region in a robotic exploration context based on sensor data.
	 *
	 * UNSEEN: 		The region has not yet been detected by the robot's sensors.
	 * FREE: 		The region has been detected by the robot's sensors, and it is known to be free of obstacles.
	 * OCCUPIED: 	The region has been detected by the robot's sensors, and it is known to contain obstacles.
	 *
	 * Note: there is an order of priority: UNSEEN, then SEEN_FREE, then SEEN_OCCUPIED.
	 */
	enum RegionType {
		UNSEEN, FREE, OCCUPIED
	};

	/**
	 * Convert the given RegionType to a string.
	 */
	static std::string region_type_to_string(RegionType type) {
		switch (type) {
			case UNSEEN:
				return "UNSEEN";
			case FREE:
				return "FREE";
			case OCCUPIED:
				return "OCCUPIED";
			default:
				return "UNKNOWN";
		}
	}

	/**
	 * @fn virtual void incorporate(const Eigen::Vector3d &eye_center, const SegmentedPointCloud &pointCloud)
	 *
	 * Incorporate a segmented point cloud into the occupancy map.
	 *
	 * @param eye_center 		The center of the robot's sensor, used as a reference point for incorporating the point cloud.
	 * @param pointCloud 		The segmented point cloud to be incorporated into the occupancy map.
	 */
	virtual void incorporate(const Eigen::Vector3d &eye_center, const SegmentedPointCloud &pointCloud) = 0;

	/**
	 * @fn virtual void incorporate(const Eigen::Vector3d &eye_center, const RegionDefinitionFn &region_fn)
	 *
	 * Incorporate a region defined by the given function into the occupancy map. The function should return the closest point
	 * on any of the boundary surfaces that currently limit the robot's vision. The bounding surfaces are assumed to be
	 * star-shaped, centered on the robot's sensor center.
	 *
	 * @param eye_center	The center of the robot's sensor, used as a reference point for incorporating the region.
	 * @param region_fn  	The function defining the region to be incorporated into the occupancy map.
	 */
	virtual void incorporate(const Eigen::Vector3d &eye_center, const RegionDefinitionFn &region_fn) = 0;

	/**
	 * @fn virtual RegionType getRegionType(const Eigen::Vector3d &point) const
	 *
	 * Query the occupancy map for the type of region at the given point.
	 *
	 * @param query_point 		The point at which to query the occupancy map.
	 * @return 					The type of region at the given point.
	 */
	virtual RegionType query_at(const Eigen::Vector3d &query_point) = 0;

	/// Destructor.
	virtual ~OccupancyMap() = default;

};

#endif //NEW_PLANNERS_OCCUPANCYMAP_H
