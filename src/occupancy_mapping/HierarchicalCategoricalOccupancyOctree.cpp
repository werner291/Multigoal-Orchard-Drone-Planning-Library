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

// The resolution at which to stop recursing and update the leaf cells
const double RESOLUTION = 0.1;

/**
 * @fn void incorporate_internal(const Eigen::AlignedBox3d &box, Cell &cell, const Eigen::Vector3d &eye_center, const SegmentedPointCloud::Point &pt)
 * A helper function for incorporate() that recursively updates the octree cells with the given point cloud data.
 *
 * @param box The bounding box of the current octree cell.
 *
 *  @param cell The current octree cell to be updated.
 *
 * @param eye_center The center of the robot's sensor, used as a reference point for incorporating the point cloud.
 *
 * @param pt The current point in the point cloud being incorporated into the octree.
 */
void incorporate_internal(const Eigen::AlignedBox3d &box,
						  Cell &cell,
						  const Eigen::Vector3d &eye_center,
						  const SegmentedPointCloud::Point &pt) {

	// Create a line segment from the eye center to the point in the point cloud
	Segment3d segment{eye_center, pt.position};

	// Check if the current cell is at the bottom of the octree (i.e. smallest resolution)
	bool at_bottom = box.sizes()[0] <= RESOLUTION;

	// Check if the line segment intersects the current cell's bounding box
	bool intersection = math_utils::segment_intersects_aabb(box, segment);

	// If the line segment does not intersect the cell, return without updating the cell
	if (!intersection) {
		return;
	}

	// If the current cell is at the bottom of the octree:
	if (at_bottom) {

		// The cell must be a leaf cell, therefore we extract the LeafCell from the variant.
		auto &leaf_cell = std::get<LeafCell>(cell);

		// - Unseen cells are updated to free cells
		if (leaf_cell.data == OccupancyMap::UNSEEN) {
			leaf_cell.data = OccupancyMap::FREE;
		}

		// For hard obstacles, we set the cell to "occupied" instead
		if (box.contains(segment.end) && pt.type == SegmentedPointCloud::PT_OBSTACLE) {
			leaf_cell.data = OccupancyMap::OCCUPIED;
		}

	} else {
		// Else, we are not at maximum resolution.

		// If the cell is a leaf cell, split it into 8 sub-cells with identical label.
		if (const auto &leaf = std::get_if<LeafCell>(&cell)) {
			cell = {CatOctree::split_by_copy(*leaf, {})};
		}

		OctantIterator iter(box);

		auto &split_cell = std::get<SplitCell>(cell);
		// Iterate over the 8 child cells
		for (auto &child_cell : *split_cell.children) {
			// Recursively update the child cells with the current point cloud data
			incorporate_internal(*iter++, child_cell, eye_center, pt);
		}

		// If all of the child cells have the same data, collapse the split cell back into a leaf cell
		if (split_cell.has_uniform_leaves()) {
			cell = std::get<LeafCell>(*split_cell.children->begin());
		}

	}
}

/**
 *
 *  @brief Recursive function to incorporate a region defined by a given function into the octree.
 *  @param box The bounding box of the current cell.
 *  @param cell The current cell to incorporate the region into.
 *  @param region_fn The function defining the region to be incorporated.
 */
void incorporate_internal(const Eigen::AlignedBox3d &box,
						  Cell &cell,
						  const RegionDefinitionFn &region_fn) {

	// Get the closest boundary point to the center of the current cell
	BoundarySample boundary_sample = region_fn(box.center());

	// Check whether the center of the cell lies inside the region, assumed to be star-shaped around the eye center point.
	// If yes, the center is currently visible to the robot's sensor.
	bool center_inside_region = (box.center() - boundary_sample.surface_point).squaredNorm() < box.diagonal().squaredNorm() / 2;

	// Check if the sample point indicates that the boundary of the region may lie inside of the cell bounds.
	bool boundary_may_lie_inside_cell = (box.center() - boundary_sample.surface_point).norm() <= box.sizes()[0] * sqrt(3);

	if (box.sizes()[0] <= RESOLUTION) {

		auto &leaf_cell = std::get<LeafCell>(cell);

		if (center_inside_region && leaf_cell.data == OccupancyMap::UNSEEN) {
			leaf_cell.data = OccupancyMap::FREE;
		}

		if (boundary_may_lie_inside_cell && boundary_sample.boundary_type == OBSTRUCTING) {
			leaf_cell.data = OccupancyMap::OCCUPIED;
		}

	}
	else {
		// Recursive case: split cell

		if (boundary_may_lie_inside_cell) {

			// Recursive case: split cell and recurse

			// If not split, do so now.
			if (auto *leaf_cell = std::get_if<LeafCell>(&cell)) {

				if (leaf_cell->data == OccupancyMap::FREE) {
					// Full cell is free; this will not change, so we can early-return.
					return;
				}

				cell = CatOctree::split_by_copy(*leaf_cell, std::monostate{});
			}

			auto &split_cell = std::get<SplitCell>(cell);

			OctantIterator iter(box);
			for (size_t octant_i = 0; octant_i < 8; ++octant_i) {
				const auto &octant = *(iter++);
				incorporate_internal(octant, (*split_cell.children)[octant_i], region_fn);
			}

			// If all the child cells have the same data, collapse the split cell back into a leaf cell
			if (split_cell.has_uniform_leaves()) {
				cell = std::get<LeafCell>(*split_cell.children->begin());
			}
		} else if (center_inside_region) {

			// If the cell lies fully inside the region, it must be free.
			cell = LeafCell {
				OccupancyMap::FREE
			};

		} else {
			// Cell lies fully outside the region, so do nothing.
		}

	}
}

void HierarchicalCategoricalOccupancyOctree::incorporate(const Eigen::Vector3d &eye_center,
														 const SegmentedPointCloud &pointCloud) {

	for (const SegmentedPointCloud::Point &pt: pointCloud.points) {
		// Update the octree cells with the current point cloud data using the incorporate_internal() helper function
		incorporate_internal(tree.box, tree.root, eye_center, pt);
	}

}

OccupancyMap::RegionType HierarchicalCategoricalOccupancyOctree::query_at(const Eigen::Vector3d &query_point) {
	return tree.get_leaf_data_at(query_point);
}


void HierarchicalCategoricalOccupancyOctree::incorporate(const Eigen::Vector3d &eye_center,
														 const RegionDefinitionFn &region_fn) {
	incorporate_internal(tree.box, tree.root, region_fn);
}
