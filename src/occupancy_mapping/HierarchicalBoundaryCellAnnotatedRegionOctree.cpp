// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-12-22.
//

#include "HierarchicalBoundaryCellAnnotatedRegionOctree.h"

HierarchicalBoundaryCellAnnotatedRegionOctree::HierarchicalBoundaryCellAnnotatedRegionOctree(const Eigen::Vector3d &center,
																							 const double baseEdgeLength)
		: center(center), base_edge_length(baseEdgeLength) {
}

void HierarchicalBoundaryCellAnnotatedRegionOctree::incorporate(const Eigen::Vector3d &eye_center,
																const SegmentedPointCloud &pointCloud) {
	throw std::runtime_error("Not implemented");
}

void HierarchicalBoundaryCellAnnotatedRegionOctree::incorporate(const Eigen::Vector3d &eye_center,
																const RegionDefinitionFn &region_fn) {
	throw std::runtime_error("Not implemented");
}

OccupancyMap::RegionType HierarchicalBoundaryCellAnnotatedRegionOctree::query_at(const Eigen::Vector3d &query_point) {
	throw std::runtime_error("Not implemented");
}
