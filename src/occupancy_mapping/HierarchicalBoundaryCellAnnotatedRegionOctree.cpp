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
		double sample_distance_to_eye = (sample.surface_point - eye_center).norm();
		double box_center_distance_to_eye = (box.center() - eye_center).norm();
		bool is_inside = box_center_distance_to_eye < sample_distance_to_eye;

		EigenExt::Plane3d new_occluding_plane(
				(sample.surface_point - box.center()).normalized() * (is_inside ? 1.0 : -1.0), sample.surface_point);

		double old_signed_distance = leaf.data.plane ? leaf.data.plane->signedDistance(box.center())
													 : std::numeric_limits<double>::infinity();
		double new_signed_distance = new_occluding_plane.signedDistance(box.center());

		if (leaf.data.region == UNSEEN || new_signed_distance > old_signed_distance) {
			leaf.data.region = FREE;

			if (abs(new_signed_distance) > sqrt(3) * box.sizes()[0] / 2.0) {
				// The new plane is too far away from the box center to be considered a boundary.
				leaf.data.plane = std::nullopt;
			} else {
				leaf.data.plane = new_occluding_plane;
			}
		}

	}, [&](const Eigen::AlignedBox3d &box, LeafCell &leaf) {

		// Return whether to split the cell. We do if a sample from the region_fn closest to the eye_center lies within sqrt(3) * edge_length of the center of the cell.

		BoundarySample sample = region_fn(box.center());

		double d = (sample.surface_point - eye_center).norm();

		return d < sqrt(3) * box.sizes()[0] / 2;
	});

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
