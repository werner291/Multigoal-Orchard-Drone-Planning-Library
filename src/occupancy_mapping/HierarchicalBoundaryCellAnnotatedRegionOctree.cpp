// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-12-22.
//

#include <range/v3/view/filter.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/transform.hpp>
#include "../utilities/math_utils.h"
#include "HierarchicalBoundaryCellAnnotatedRegionOctree.h"

bool isPointInsideViewPyramid(const math_utils::ViewPyramidFaces &planes, const Eigen::Vector3d &center) {
	for (const auto &plane : {planes.bottom, planes.top, planes.left, planes.right}) {
		if ((center-plane.apex).dot(*plane.normal()) < 0) {
			return false;
		}
	}

	return true;
}

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

	// If the cell is fully-seen, we can stop here.
	if (cell.is_leaf() && cell.get_leaf().data.isUniform() && cell.get_leaf().data.get_uniform_cell().seen) {
		return;
	}

	// Also, go find the points whose eye ray passes through the cell at all. This is essentially pre-filtering step on the input.
	std::vector<OccupancyMap::OccludingPoint> affecting_points = candidate_occluding_points | ranges::views::filter([&](const auto &p) {
		return math_utils::intersects(box, math_utils::Ray3d(eye_transform.translation(), p.point - eye_transform.translation()));
	}) | ranges::to_vector;

	// Compute the delimiting planes of the view pyramid.
	auto planes = math_utils::compute_view_pyramid_planes(eye_transform, fovX, fovY);

	// Check whether the cell bounding box intersects the view pyramid planes.
	Eigen::Vector3d center = box.center();

	// Sample the points closest to the cell center on the view pyramid planes.
	std::array<Eigen::Vector3d, 4> sample_points {
		closest_point_on_open_triangle(center, planes.bottom),
		closest_point_on_open_triangle(center, planes.top),
		closest_point_on_open_triangle(center, planes.left),
		closest_point_on_open_triangle(center, planes.right)
	};

	bool may_intersect_planes = false;
	for (const auto &sample_point : sample_points) {
		if ((sample_point - center).norm() < box.sizes().norm() / 2) {
			may_intersect_planes = true;
			break;
		}
	}

	bool points_inside = std::any_of(candidate_occluding_points.begin(), candidate_occluding_points.end(),
									 [&box](const auto &point) {
										 return box.contains(point.point);
									 });

	bool should_split = (points_inside || may_intersect_planes) && maxDepth > 0;

	if (cell.is_leaf() && should_split) {
		cell.split_by_copy({});
	}

	if (cell.is_split()) {

		SplitCell &split_cell = cell.get_split();

		OctantIterator octant_iterator(box);

		for (Cell &child: *split_cell.children) {
			// Recurse.
			incorporate_internal(*(octant_iterator++),
								 child,
								 affecting_points,
								 maxDepth - 1,
								 eye_transform,
								 fovX,
								 fovY);
		}

	} else {

		// Check if any of the points fully occlude this cell. If so, leave untouched.
		// A point fully occludes the cell if the ray cast from the point away from the eye center
		// intersects the cell, without the point itself being inside the cell.
		bool fully_occluded = std::any_of(affecting_points.begin(), affecting_points.end(), [&](const auto &point) {
			return math_utils::intersects(box, math_utils::Ray3d(point.point, eye_transform.translation() - point.point)) &&
				   !box.contains(point.point);
		});

		if (fully_occluded) {
			return;
		}

		// Now, check if the cell is fully completely outside of the view pyramid.
		// If it is, then it is invisible, and we also early-return.
		if (isPointInsideViewPyramid(planes, center) && !may_intersect_planes) {
			return;
		}

		// If the cell cannot intersect the planes, and contains no occluding points, then it is fully visible.
		if (!may_intersect_planes && !points_inside) {
			cell.cell = LeafCell {
				.data = {HierarchicalBoundaryCellAnnotatedRegionOctree::UniformCell {
					.seen = true
				}}
			};
			return;
		}

		// By De-Morgan's law: either a plane intersects the cell, or the cell contains a point.

		// We'll want to find whichever one expands the cell the *least*, since we can always expand the cell, but never contract it.

		EigenExt::Plane3d closest_point;
		bool closest_point_is_hard = false;
		double closest_sd = std::numeric_limits<double>::infinity();

		for (const auto& plane: {planes.bottom, planes.top, planes.left, planes.right}) {
			Eigen::Vector3d point = closest_point_on_open_triangle(center, plane);
			EigenExt::Plane3d point_plane = EigenExt::Plane3d((point - center).normalized(), point);
			double sd = (point - center).dot(*plane.normal());

			if (sd < closest_sd) {
				closest_sd = sd;
				closest_point = point_plane;
				closest_point_is_hard = false;
			}
		}

		for (const auto &point: candidate_occluding_points) {
			// If the point is outside the cell, ignore it.
			if (!box.contains(point.point)) {
				continue;
			}

			EigenExt::Plane3d point_plane = EigenExt::Plane3d((point.point - center).normalized(), point.point);

			// If the normal points away from the eye center, flip it.
			if (point_plane.normal().dot(point.point - eye_transform.translation()) > 0) {
				point_plane.normal() *= -1;
			}

			double sd = (point.point - center).dot(point_plane.normal());

			if (sd < closest_sd) {
				closest_sd = sd;
				closest_point = point_plane;
				closest_point_is_hard = point.hard;
			}

		}

		assert(cell.is_leaf());

		// If the cell is fully-seen, we can stop here.
		if (cell.get_leaf().data.isUniform() && cell.get_leaf().data.get_uniform_cell().seen) {
			return;
		}

		// If the existing point is harder than the new point, we can stop here.
		if (cell.get_leaf().data.isBoundary() && cell.get_leaf().data.get_boundary_cell().hard && !closest_point_is_hard) {
			return;
		}

		// If the cell is uniform and unseen update now and return.
		if (cell.get_leaf().data.isUniform() && !cell.get_leaf().data.get_uniform_cell().seen) {
			cell.cell = LeafCell {
				.data = {HierarchicalBoundaryCellAnnotatedRegionOctree::BoundaryCell {
						.plane = closest_point,
						.hard = closest_point_is_hard,
				}}
			};
			return;
		}

		// Extract the plane from the cell.
		EigenExt::Plane3d existing_plane = cell.get_leaf().data.get_boundary_cell().plane;
		double old_sd = existing_plane.signedDistance(center);

		if (old_sd < closest_sd) {
			return;
		}

		cell.cell = LeafCell {
			.data = {HierarchicalBoundaryCellAnnotatedRegionOctree::BoundaryCell {
					.plane = closest_point,
					.hard = closest_point_is_hard,
			}}
		};


	}

}

void HierarchicalBoundaryCellAnnotatedRegionOctree::incorporate(const std::vector<OccupancyMap::OccludingPoint> &occluding_points,
																const Eigen::Isometry3d &eye_transform,
																double fovX,
																double fovY) {

	auto planes = math_utils::compute_view_pyramid_planes(eye_transform, fovX, fovY);

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

bool HierarchicalBoundaryCellAnnotatedRegionOctree::LeafData::isBoundary() const {
	return std::holds_alternative<BoundaryCell>(data);
}
