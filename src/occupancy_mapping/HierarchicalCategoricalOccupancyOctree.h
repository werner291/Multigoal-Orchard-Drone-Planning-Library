// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-12-22.
//

#ifndef NEW_PLANNERS_HIERARCHICALCATEGORICALOCCUPANCYOCTREE_H
#define NEW_PLANNERS_HIERARCHICALCATEGORICALOCCUPANCYOCTREE_H

#include "Octree.h"
#include "OccupancyMap.h"
#include "VisibilityBoundary.h"

/**
 * A simple octree with categorical leaf cells, as opposed to
 */
class HierarchicalCategoricalOccupancyOctree : public OccupancyMap {

public:
	void incorporate(const Eigen::Vector3d &eye_center, const SegmentedPointCloud &pointCloud) override;

	RegionType query_at(const Eigen::Vector3d &query_point) override;

	using CategoricalOctree = Octree<std::monostate, RegionType>;

	CategoricalOctree tree{OccupancyMap::RegionType::UNSEEN};

};

#endif //NEW_PLANNERS_HIERARCHICALCATEGORICALOCCUPANCYOCTREE_H
