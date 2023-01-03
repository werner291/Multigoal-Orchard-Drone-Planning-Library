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
						  const std::vector<OccupancyMap::OccludingPoint> &candidate_occluding_points,
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
	std::vector<OccupancyMap::OccludingPoint> occluding_points = candidate_occluding_points | ranges::views::filter([&](const auto &p) {
		return math_utils::intersects(box, math_utils::Ray3d(p.point, p.point - eye_transform.translation()));
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

		for (const auto &p : occluding_points) {

			Eigen::Vector3d normal = (p.point - box.center()).normalized();

			if (normal.dot(eye_transform.translation() - p.point) < 0) {
				normal = -normal;
			}

			leaf_cell.data.updateBoundary(EigenExt::Plane3d(normal, p.point), p.hard, box.center());
		}

	}

}

void HierarchicalBoundaryCellAnnotatedRegionOctree::incorporate(const std::vector<OccupancyMap::OccludingPoint> &occluding_points,
																const Eigen::Isometry3d &eye_transform,
																double fovX,
																double fovY) {

	incorporate_internal(tree.box, tree.root, occluding_points, max_depth, eye_transform, fovX, fovY);

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
