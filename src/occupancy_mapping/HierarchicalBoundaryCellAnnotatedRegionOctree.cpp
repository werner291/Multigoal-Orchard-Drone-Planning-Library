// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-12-22.
//

#include "HierarchicalBoundaryCellAnnotatedRegionOctree.h"

using PtOctree = HierarchicalBoundaryCellAnnotatedRegionOctree::PointAnnotatedOctree;
using LeafCell = PtOctree::LeafCell;
using SplitCell = PtOctree::SplitCell;
using Cell = PtOctree::Cell;

HierarchicalBoundaryCellAnnotatedRegionOctree::HierarchicalBoundaryCellAnnotatedRegionOctree(const Eigen::Vector3d &center,
																							 const double baseEdgeLength, const unsigned int maxDepth)
		: max_depth(maxDepth), tree(LeafData {
		.region = UNSEEN,
		.plane = std::nullopt
		}) {

	// Initialize the octree bounding box with the given center and base edge length
	tree.box = Eigen::AlignedBox3d(center - Eigen::Vector3d(baseEdgeLength / 2, baseEdgeLength / 2, baseEdgeLength / 2),
								   center + Eigen::Vector3d(baseEdgeLength / 2, baseEdgeLength / 2, baseEdgeLength / 2));

}

void HierarchicalBoundaryCellAnnotatedRegionOctree::incorporate(const Eigen::Vector3d &eye_center,
																const SegmentedPointCloud &pointCloud) {

	throw std::runtime_error("Not implemented");

}

void HierarchicalBoundaryCellAnnotatedRegionOctree::incorporate(const Eigen::Vector3d &eye_center,
																const RegionDefinitionFn &region_fn) {

	tree.traverse(max_depth, [&](const Eigen::AlignedBox3d &box, LeafCell &leaf) {
		// We're at the highest LOD. Sample the region function at the center of the cell.

		/*
		 * What we want to do, is compare the data in the leaf cell with the boundary sample, and store
		 * the boundary sample if it's either "stronger" (hard obstacle vs occluding obstacle) or,
		 * if it's the same, whichever has a lower signed distance to the box center.
		 */

		BoundarySample sample = region_fn(box.center());

		assert(sample.boundary_type == OCCLUDING); // Save ourselves the headache: we only support occluding boundaries for now.
		assert(leaf.data.region != OCCUPIED); // Also, we don't look at occupied cells yet.

		// Assuming the observed region is star-shaped, determine whether the box center is inside or outside the region.
		// TODO: If this proves unreliable, we can also simply add an inside/outside flag to the BoundarySample struct.
		double sample_distance_to_eye = (sample.surface_point - eye_center).norm();
		double box_center_distance_to_eye = (box.center() - eye_center).norm();
		double distance_to_center = (sample.surface_point - box.center()).norm();

		bool cell_may_cross_boundary = distance_to_center <= box.sizes()[0] * sqrt(3)/2.0;

		bool is_inside = box_center_distance_to_eye < sample_distance_to_eye;

		enum LeafCases {
			WHOLE_UNSEEN = 0,
			PARTIAL_SEEN = 1,
			FULLY_SEEN = 2,
		};

		LeafCases old_leaf_case;

		if (leaf.data.region == UNSEEN) {
			assert(!leaf.data.plane.has_value());
			old_leaf_case = WHOLE_UNSEEN;
		} else if (leaf.data.region == FREE) {
			if (leaf.data.plane.has_value()) {
				old_leaf_case = PARTIAL_SEEN;
			} else {
				old_leaf_case = FULLY_SEEN;
			}
		}

		EigenExt::Plane3d new_occluding_plane((sample.surface_point - box.center()).normalized() * (is_inside ? -1.0 : 1.0), sample.surface_point);

		LeafCases new_leaf_case;

		if (cell_may_cross_boundary) {
			// The cell may cross the boundary. We need to split it.
			new_leaf_case = PARTIAL_SEEN;
		} else {
			if (is_inside) {
				new_leaf_case = FULLY_SEEN;
			} else {
				new_leaf_case = WHOLE_UNSEEN;
			}
		}

		if (new_leaf_case > old_leaf_case) {
			// The new sample is stronger than the old one. Replace it.
			leaf.data.region = FREE;
			leaf.data.plane = (cell_may_cross_boundary) ? std::optional(new_occluding_plane) : std::nullopt;
		} else if (new_leaf_case == old_leaf_case) {

			switch (new_leaf_case) {
				case WHOLE_UNSEEN:
					// Nothing to do.
					break;
				case PARTIAL_SEEN:
								// The new sample is as strong as the old one. Keep the one that's closer to the box center.
								if (new_occluding_plane.signedDistance(box.center()) > leaf.data.plane->signedDistance(box.center())) {
									leaf.data.region = FREE;
									leaf.data.plane = new_occluding_plane;
								}
					break;
				case FULLY_SEEN:
					leaf.data.region = FREE;
					leaf.data.plane = std::nullopt;
					break;
			}


		}

	}, SplitIfBoundaryMaybeInsideCell<LeafCell>{eye_center, region_fn});

}

OccupancyMap::RegionType HierarchicalBoundaryCellAnnotatedRegionOctree::query_at(const Eigen::Vector3d &query_point) {
	const LeafData &data = tree.get_leaf_data_at(query_point);

	assert(data.region != OCCUPIED); // We don't look at occupied cells yet.

	if (data.plane) {

		double sd = data.plane->signedDistance(query_point);

		return sd > 0.0 ? FREE : UNSEEN;

	} else {
		return data.region;
	}

}

const HierarchicalBoundaryCellAnnotatedRegionOctree::PointAnnotatedOctree &
HierarchicalBoundaryCellAnnotatedRegionOctree::getTree() const {
	return tree;
}

const unsigned int HierarchicalBoundaryCellAnnotatedRegionOctree::getMaxDepth() const {
	return max_depth;
}
