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


/**
 *
 *  @brief Recursive function to incorporate a region defined by a given function into the octree.
 *  @param box The bounding box of the current cell.
 *  @param cell The current cell to incorporate the region into.
 *  @param region_fn The function defining the region to be incorporated.
 */
void incorporate_internal(const Eigen::AlignedBox3d &box,
						  Cell &cell,
						  const RegionDefinitionFn &region_fn,
						  const int maxDepth) {

	// Get the closest boundary point to the center of the current cell
	BoundarySample boundary_sample = region_fn(box.center());

	// Check whether the center of the cell lies inside the region, assumed to be star-shaped around the eye center point.
	// If yes, the center is currently visible to the robot's sensor.
	bool center_inside_region =
			(box.center() - boundary_sample.surface_point).squaredNorm() < box.diagonal().squaredNorm() / 2;

	// Check if the sample point indicates that the boundary of the region may lie inside of the cell total_box.
	bool boundary_may_lie_inside_cell =
			(box.center() - boundary_sample.surface_point).norm() <= box.sizes()[0] * sqrt(3);

	// If we are at the highest LOD, we can simply incorporate the boundary sample into the cell.
	if (maxDepth == 0) {

		// Get the leaf cell data from the variant.
		auto &leaf_cell = cell.get_leaf();

		// If the center of the cell lies inside the region, the cell is currently visible to the robot's sensor.
		if (center_inside_region && leaf_cell.data == OccupancyMap::UNSEEN) {
			leaf_cell.data = OccupancyMap::FREE;
		}

		// If the boundary of the region may lie inside of the cell, and the boundary is a hard boundary,
		// the cell is occupied by a hard obstacle.
		if (boundary_may_lie_inside_cell && boundary_sample.boundary_type == OBSTRUCTING) {
			leaf_cell.data = OccupancyMap::OCCUPIED;
		}

	} else {

		// Recursive case: split cell
		if (boundary_may_lie_inside_cell) {

			// Recursive case: split cell and recurse

			// If not split, do so now.
			if (auto *leaf_cell = std::get_if<LeafCell>(&cell.cell)) {

				if (leaf_cell->data == OccupancyMap::FREE) {
					// Full cell is free; this will not change, so we can early-return.
					return;
				}

				cell.split_by_copy({});
			}

			auto &split_cell = cell.get_split();

			OctantIterator iter(box);
			for (size_t octant_i = 0; octant_i < 8; ++octant_i) {
				const auto &octant = *(iter++);
				incorporate_internal(octant, (*split_cell.children)[octant_i], region_fn, maxDepth - 1);
			}

			// If all the child cells have the same data, collapse the split cell back into a leaf cell
			if (split_cell.has_uniform_leaves()) {
				cell.merge_cell();
			}

		} else if (center_inside_region) {

			// If the cell lies fully inside the region, it must be free.
			cell.cell = LeafCell{OccupancyMap::FREE};

		} else {
			// Cell lies fully outside the region, so do nothing.
		}

	}
}

void HierarchicalCategoricalOccupancyOctree::incorporate(const SegmentedPointCloud &pointCloud,
														 const Eigen::Isometry3d &eye_transform,
														 double fovX,
														 double fovY) {

	for (const SegmentedPointCloud::Point &pt: pointCloud.points) {

		// Create a line segment from the eye center to the point in the point cloud
		math_utils::Segment3d segment{eye_transform.translation(), pt.position};

		// Update the octree cells with the current point cloud data using the incorporate_internal() helper function
		tree.traverse(maxDepth, [&](const Eigen::AlignedBox3d &box, LeafCell &leaf_cell) {
			// - Unseen cells are updated to free cells
			if (leaf_cell.data == OccupancyMap::UNSEEN) {
				leaf_cell.data = OccupancyMap::FREE;
			}

			// For hard obstacles, we set the cell to "occupied" instead
			if (box.contains(segment.end) && pt.type == SegmentedPointCloud::PT_OBSTACLE) {
				leaf_cell.data = OccupancyMap::OCCUPIED;
			}
		}, [&](const Eigen::AlignedBox3d &box, LeafCell &leaf_cell) {
			return math_utils::segment_intersects_aabb(box, segment);
		});
	}

}

OccupancyMap::RegionType HierarchicalCategoricalOccupancyOctree::query_at(const Eigen::Vector3d &query_point) const {
	return tree.get_leaf_data_at(query_point);
}


void HierarchicalCategoricalOccupancyOctree::incorporate(const Eigen::Vector3d &eye_center,
														 const RegionDefinitionFn &region_fn) {

	tree.traverse(maxDepth, [&](const Eigen::AlignedBox3d &box, LeafCell &leaf_cell) {

		// Split a cell if the boundary of the region may lie inside of it.
		BoundarySample boundary_sample = region_fn(box.center());

		double eye_sample_distance = (eye_center - boundary_sample.surface_point).norm();
		double center_eye_distance = (box.center() - eye_center).norm();

		if (center_eye_distance < eye_sample_distance) {
			leaf_cell.data = OccupancyMap::FREE;
		}

	}, SplitIfBoundaryMaybeInsideCell<LeafCell>{eye_center, region_fn});

}

HierarchicalCategoricalOccupancyOctree::HierarchicalCategoricalOccupancyOctree(const Eigen::Vector3d &center,
																			   const double baseEdgeLength,
																			   const int maxDepth)
		: maxDepth(maxDepth) {

	// Initialize the octree bounding box with the given center and base edge length
	tree.box = Eigen::AlignedBox3d(center - Eigen::Vector3d(baseEdgeLength / 2, baseEdgeLength / 2, baseEdgeLength / 2),
								   center +
								   Eigen::Vector3d(baseEdgeLength / 2, baseEdgeLength / 2, baseEdgeLength / 2));

	// Initialize the root cell to be a leaf cell with the default value
	tree.root.cell = LeafCell{OccupancyMap::UNSEEN};


}

double HierarchicalCategoricalOccupancyOctree::getMinEdgeLength() const {
	return tree.box.sizes()[0] / pow(2, maxDepth);
}

const HierarchicalCategoricalOccupancyOctree::CategoricalOctree &
HierarchicalCategoricalOccupancyOctree::getOctree() const {
	return tree;
}
