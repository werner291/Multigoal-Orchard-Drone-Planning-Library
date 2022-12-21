// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-12-22.
//

#include "HierarchicalCategoricalOccupancyOctree.h"
#include "../utilities/Grid.h"
#include "../utilities/math_utils.h"

using CatOctree = HierarchicalCategoricalOccupancyOctree::CategoricalOctree;
using LeafCell = CatOctree::LeafCell;
using SplitCell = CatOctree::SplitCell;
using Cell = CatOctree::Cell;

void incorporate_internal(const Eigen::AlignedBox3d &box,
						  Cell &cell,
						  const Eigen::Vector3d &eye_center,
						  const SegmentedPointCloud::Point &pt) {

	const double RESOLUTION = 0.1;

	Segment3d segment{eye_center, pt.position};

	bool at_bottom = box.sizes()[0] <= RESOLUTION;
	bool intersection = math_utils::segment_intersects_aabb(box, segment);

	if (!intersection) {
		return;
	}

	if (at_bottom) {

		auto &leaf_cell = std::get<LeafCell>(cell);
		if (leaf_cell.data == OccupancyMap::UNSEEN) {
			leaf_cell.data = OccupancyMap::FREE;
		}
		if (box.contains(segment.end) && pt.type == SegmentedPointCloud::PT_OBSTACLE) {
			leaf_cell.data = OccupancyMap::OCCUPIED;
		}


	} else {

		if (const auto &leaf = std::get_if<LeafCell>(&cell)) {
			cell = {CatOctree::split_by_copy(*leaf, {})};
		}
		OctantIterator iter(box);
		auto &split_cell = std::get<SplitCell>(cell);
		for (size_t octant_i = 0; octant_i < 8; ++octant_i) {
			incorporate_internal(*(iter++), split_cell.children->operator[](octant_i), eye_center, pt);
		}
		if (split_cell.has_uniform_leaves()) {
			cell = std::get<LeafCell>(*split_cell.children->begin());
		}

	}
}

void HierarchicalCategoricalOccupancyOctree::incorporate(const Eigen::Vector3d &eye_center,
														 const SegmentedPointCloud &pointCloud) {


	for (const SegmentedPointCloud::Point &pt: pointCloud.points) {


	}

	OccupancyMap::RegionType HierarchicalCategoricalOccupancyOctree::query_at(const Eigen::Vector3d &query_point) {
		return OCCUPIED;
	}
