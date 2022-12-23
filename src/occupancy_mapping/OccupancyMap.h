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

	virtual void incorporate(const Eigen::Vector3d &eye_center, const SegmentedPointCloud &pointCloud) = 0;

	virtual void incorporate(const RegionDefinitionFn& region_fn) = 0;

	virtual RegionType query_at(const Eigen::Vector3d &query_point) = 0;

};

#endif //NEW_PLANNERS_OCCUPANCYMAP_H
