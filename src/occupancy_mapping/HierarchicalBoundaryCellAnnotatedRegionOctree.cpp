// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-12-22.
//

#include <range/v3/view/filter.hpp>
#include <range/v3/range/conversion.hpp>
#include "HierarchicalBoundaryCellAnnotatedRegionOctree.h"
#include "../utilities/math_utils.h"

using PtOctree = HierarchicalBoundaryCellAnnotatedRegionOctree::PointAnnotatedOctree;
using LeafCell = PtOctree::LeafCell;
using SplitCell = PtOctree::SplitCell;
using Cell = PtOctree::Cell;

HierarchicalBoundaryCellAnnotatedRegionOctree::HierarchicalBoundaryCellAnnotatedRegionOctree(const Eigen::Vector3d &center,
																							 const double baseEdgeLength,
																							 const unsigned int maxDepth)
		: max_depth(maxDepth), tree(LeafData{.data = UniformCell {
			.seen = false
		}}) {

	// Initialize the octree bounding box with the given center and base edge length
	tree.box = Eigen::AlignedBox3d(center - Eigen::Vector3d(baseEdgeLength / 2, baseEdgeLength / 2, baseEdgeLength / 2),
								   center +
								   Eigen::Vector3d(baseEdgeLength / 2, baseEdgeLength / 2, baseEdgeLength / 2));

}

/**
 * @brief Recursive function to incorporate a point cloud into the octree.
 *
 * @param box The bounding box of the current cell.
 * @param cell The current cell to incorporate the point cloud into.
 * @param candidate_occluding_points Vector of points to be incorporated; to be interpreted as rays from the eye_center
 * @param maxDepth The maximum depth of the octree.
 * @param eye_tranform The transform of the eye used to view the point cloud.
 * @param fovX The horizontal field of view of the eye (radians)
 * @param fovY The vertical field of view of the eye (radians)
 */
void incorporate_internal(const Eigen::AlignedBox3d &box,
						  Cell &cell,
						  const std::vector<SegmentedPointCloud::Point> &candidate_occluding_points,
						  const int maxDepth,
						  const Eigen::Isometry3d &eye_transform,
						  double fovX,
						  double fovY) {

	// Compute the delimiting planes of the view pyramid.
	auto planes = math_utils::compute_view_pyramid_planes(eye_transform, fovX, fovY);

	// Check whether the cell bounding box intersects the view pyramid planes.
	bool bottom_intersects = math_utils::intersects(box, planes.bottom);
	bool top_intersects = math_utils::intersects(box, planes.top);
	bool left_intersects = math_utils::intersects(box, planes.left);
	bool right_intersects = math_utils::intersects(box, planes.right);

	int view_pyramid_intersections = bottom_intersects + top_intersects + left_intersects + right_intersects;

	// Check if the cell center is inside the view pyramid.
	bool cell_center_inside_view_pyramid =
			planes.bottom.signedDistance(box.center()) > 0 &&
			planes.top.signedDistance(box.center()) > 0 &&
			planes.left.signedDistance(box.center()) > 0 &&
			planes.right.signedDistance(box.center()) > 0;

	// Check if the cell is completely outside the view pyramid.
	bool lies_fully_outside = view_pyramid_intersections == 0 && !cell_center_inside_view_pyramid;

	if (lies_fully_outside) {
		// The cell is completely out of view; don't touch it.
		return;
	}

	// Else, at least some point of the cell lies inside the view pyramid.

	// Then, find out which of the given candidate occluding points are actually occluding some part of the cell.
	// That is: the ray from the point away from the eye center intersects the cell.
	std::vector<SegmentedPointCloud::Point> occluding_points = candidate_occluding_points | ranges::views::filter([&](const SegmentedPointCloud::Point &p) {
		return math_utils::intersects(box, math_utils::Ray3d(p.position, p.position - eye_transform.translation()));
	}) | ranges::to_vector;

	bool should_split = maxDepth > 0 && (view_pyramid_intersections > 1 || !occluding_points.empty());

	if (cell.is_leaf() && should_split) {
		cell.split_by_copy({});
	}

	if (cell.is_split()) {
		SplitCell &split_cell = cell.get_split();

		OctantIterator octant_iterator(box);

		for (Cell& child : *split_cell.children) {
			// Recurse.
			incorporate_internal(*(octant_iterator++), child, occluding_points, maxDepth - 1, eye_transform, fovX, fovY);
		}
	} else {

		// We are in the leaf case.
		// We know that at least some part of the cell lies inside the view pyramid.

		LeafCell &leaf_cell = cell.get_leaf();

		if (bottom_intersects) {
			leaf_cell.data.updateBoundary(planes.bottom, false, box.center());
		}

		if (top_intersects) {
			leaf_cell.data.updateBoundary(planes.top, false, box.center());
		}

		if (left_intersects) {
			leaf_cell.data.updateBoundary(planes.left, true, box.center());
		}

		if (right_intersects) {
			leaf_cell.data.updateBoundary(planes.right, true, box.center());
		}

		for (const SegmentedPointCloud::Point &p : occluding_points) {

			Eigen::Vector3d normal = (p.position - box.center()).normalized();

			if (normal.dot(eye_transform.translation() - p.position) < 0) {
				normal = -normal;
			}

			leaf_cell.data.updateBoundary(EigenExt::Plane3d(normal, p.position), p.type == SegmentedPointCloud::PointType::PT_OBSTACLE, box.center());
		}

	}

}

void HierarchicalBoundaryCellAnnotatedRegionOctree::incorporate(const SegmentedPointCloud &pointCloud,
																const Eigen::Isometry3d &eye_transform,
																double fovX,
																double fovY) {



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

		assert(sample.boundary_type ==
			   OCCLUDING); // Save ourselves the headache: we only support occluding boundaries for now.
		assert(leaf.data.region != OCCUPIED); // Also, we don't look at occupied cells yet.

		// Assuming the observed region is star-shaped, determine whether the box center is inside or outside the region.
		// TODO: If this proves unreliable, we can also simply add an inside/outside flag to the BoundarySample struct.
		double sample_distance_to_eye = (sample.surface_point - eye_center).norm();
		double box_center_distance_to_eye = (box.center() - eye_center).norm();
		double distance_to_center = (sample.surface_point - box.center()).norm();

		bool cell_may_cross_boundary = distance_to_center <= box.sizes()[0] * sqrt(3) / 2.0;

		bool is_inside = box_center_distance_to_eye < sample_distance_to_eye;

		enum LeafCases {
			WHOLE_UNSEEN = 0, PARTIAL_SEEN = 1, FULLY_SEEN = 2,
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

		EigenExt::Plane3d new_occluding_plane(
				(sample.surface_point - box.center()).normalized() * (is_inside ? -1.0 : 1.0), sample.surface_point);

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
					if (new_occluding_plane.signedDistance(box.center()) >
						leaf.data.plane->signedDistance(box.center())) {
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

OccupancyMap::RegionType HierarchicalBoundaryCellAnnotatedRegionOctree::query_at(const Eigen::Vector3d &query_point) const {
	const LeafData &data = tree.get_leaf_data_at(query_point);

	if (data.isUniform()) {
		return data.get_uniform_cell().seen ? FREE : UNSEEN;
	} else {
		double sd = data.get_boundary_cell().plane.signedDistance(query_point);

		return (sd > 0) ? FREE : UNSEEN;
	}

}

const HierarchicalBoundaryCellAnnotatedRegionOctree::PointAnnotatedOctree &
HierarchicalBoundaryCellAnnotatedRegionOctree::getTree() const {
	return tree;
}

const unsigned int HierarchicalBoundaryCellAnnotatedRegionOctree::getMaxDepth() const {
	return max_depth;
}

bool HierarchicalBoundaryCellAnnotatedRegionOctree::LeafData::isUniform() const {
	return std::holds_alternative<UniformCell>(data);
}

void HierarchicalBoundaryCellAnnotatedRegionOctree::LeafData::setFullySeen() {
	data = UniformCell{true};
}

void HierarchicalBoundaryCellAnnotatedRegionOctree::LeafData::updateBoundary(const Plane3d &plane,
																			 bool hard,
																			 const Eigen::Vector3d &cell_center) {

	if (isUniform()) {
		auto uniform_cell = std::get<UniformCell>(data);
		if (!uniform_cell.seen) {
			data = BoundaryCell{plane, hard};
		}
	} else {
		// It's a boundary cell, so we need to update the boundary.
		auto &boundary_cell = std::get<BoundaryCell>(data);

		if (boundary_cell.hard && !hard) {
			// Hard boundary takes precedence over soft boundary.
			return;
		}

		if (!boundary_cell.hard && hard) {
			// Hard boundary takes precedence over soft boundary.
			boundary_cell.plane = plane;
			boundary_cell.hard = hard;
			return;
		}

		// Same type of boundary, so we update based on whichever expands the seen space the most.
		double old_sd = boundary_cell.plane.signedDistance(cell_center);
		double new_sd = plane.signedDistance(cell_center);

		if (new_sd > old_sd) {
			boundary_cell.plane = plane;
		}
	}




}
