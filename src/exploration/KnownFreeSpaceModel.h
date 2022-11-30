// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_KNOWNFREESPACEMODEL_H
#define NEW_PLANNERS_KNOWNFREESPACEMODEL_H

#include <octomap/OcTree.h>
#include "SegmentedPointCloud.h"

// TODO, idea: track the BOUNDARY of free space using points!

class KnownFreeSpaceModel {

	octomap::OcTree occupancy;

public:

	KnownFreeSpaceModel() : occupancy(0.1) {
	}

	void integrate(SegmentedPointCloud::ByType &point_cloud, const Eigen::Vector3d &robot_position);

};

#endif //NEW_PLANNERS_KNOWNFREESPACEMODEL_H
