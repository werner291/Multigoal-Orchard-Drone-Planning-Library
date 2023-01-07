// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <range/v3/view/filter.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/transform.hpp>
#include "../utilities/math_utils.h"
#include "HierarchicalBoundaryCellAnnotatedRegionOctree.h"

using PtOctree = HierarchicalBoundaryCellAnnotatedRegionOctree::PointAnnotatedOctree;
using LeafCell = PtOctree::LeafCell;
using SplitCell = PtOctree::SplitCell;
using Cell = PtOctree::Cell;

HierarchicalBoundaryCellAnnotatedRegionOctree::HierarchicalBoundaryCellAnnotatedRegionOctree(const Eigen::Vector3d &center,
																							 const double baseEdgeLength,
																							 const unsigned int maxDepth)
		: max_depth(maxDepth), tree(LeafData{.data = UniformCell{.seen = false}}) {

	// Initialize the octree bounding box with the given center and base edge length
	Eigen::Vector3d half_extent(baseEdgeLength / 2, baseEdgeLength / 2, baseEdgeLength / 2);
	tree.box = Eigen::AlignedBox3d(center - half_extent,center + half_extent);
}

/**
 * Creates a linear approximation of the surface passing through a given surface sample point, by fitting a plane to the sample and the center of a bounding box.
 * The plane is oriented such that it faces a given eye point.
 *
 * @param box The bounding box to use for surface approximation.
 * @param sample A surface sample point on the plane.
 * @param eye The point towards which the plane should be oriented.
 * @return The created plane, as an approximation of the surface.
 */
EigenExt::Plane3d plane_from_sample(const Eigen::AlignedBox3d &box,
									const Eigen::Vector3d &point,
									const Eigen::Vector3d &eye) {

	// The surface normal is the vector from the sample point to the center of the bounding box, normalized.
	// with enough samples, this should be a good approximation of the surface normal.
	Eigen::Vector3d normal = (point - box.center()).normalized();

	// If the plane is not facing the eye, flip it
	if ((eye - point).dot(normal) < 0) {
		normal = -normal;
	}

	// Create the plane and return it.
	return { normal, point };
}

/**
 *
 * Determines whether or not the new plane with the given type should be used to replace the old plane.
 *
 * @param box				The bounding box of the cell.
 * @param plane				The new plane.
 * @param boundaryType		The type of the new plane.
 * @param boundaryCell		The old cell data; assumed to be a boundary cell.
 * @return					True if the new plane should replace the old plane, false otherwise.
 */
bool shouldUpdateBoundaryCell(const Eigen::AlignedBox3d &box,
							  const Plane3d &plane,
							  const HierarchicalBoundaryCellAnnotatedRegionOctree::BoundaryType &boundaryType,
							  const HierarchicalBoundaryCellAnnotatedRegionOctree::BoundaryCell &boundaryCell) {

	if (boundaryType != boundaryCell.boundaryType) {
		return boundaryType > boundaryCell.boundaryType;
	}

	// If the boundary type is a view pyramid plane:
	if (boundaryType == HierarchicalBoundaryCellAnnotatedRegionOctree::VIEW_PYRAMID_PLANE) {
		return plane.absDistance(box.center()) < boundaryCell.plane.absDistance(box.center());
	} else {
		return plane.signedDistance(box.center()) > boundaryCell.plane.signedDistance(box.center());
	}

}

/**
 * Updates leaf data of an octree with information about a boundary that passes through the leaf cell.
 *
 * @param box 			The bounding box of the octree leaf.
 * @param data 			The leaf data to update.
 * @param plane 		A linear approximation of the surface within the leaf, as created by the `plane_from_sample` function.
 * @param boundaryType 	The type of boundary represented by the plane.
 */
void updateLeafData(const Eigen::AlignedBox3d &box,
					HierarchicalBoundaryCellAnnotatedRegionOctree::LeafData &data,
					const EigenExt::Plane3d &plane,
					const HierarchicalBoundaryCellAnnotatedRegionOctree::BoundaryType &boundaryType) {
	
	// Don't touch fully-seen cells.
	if (data.isUniform()) {

		if (!data.get_uniform_cell().seen) {
			// Fully unseen cells are updated with the new plane.
			data.setBoundaryCell(plane, boundaryType);
		}

	} else {
		auto boundaryCell = data.get_boundary_cell();

		if (shouldUpdateBoundaryCell(box, plane, boundaryType, boundaryCell)) {
			// Update the leaf data with the new boundary cell information.
			data.setBoundaryCell(plane, boundaryType);
		}

		// Now, if the plane no longer intersects the cell, we need to mark it as fully-seen.
		if (!math_utils::intersects(box, boundaryCell.plane)) {
			data.setFullySeen();
		}
	}
}

/**
 * If any surface sample lies between the given eye point and the given cell,
 * we must conclude that the cell is not fully visible, and should not be changed
 * based on current data.
 *
 * This method checks whether such a sample exists.
 *
 * @param box 				The bounding box of the cell.
 * @param affecting_points 	The set of surface samples that may affect the cell.
 * @param eye 				The eye point.
 *
 * @return 					True if a surface sample lies between the eye point and the cell, false otherwise.
 */
bool checkPointsFullyOccludingCell(const Eigen::AlignedBox3d &box,
								   const std::vector<OccupancyMap::OccludingPoint> &affecting_points,
								   const Eigen::Vector3d &eye_center) {
	// Check if any point in the list intersects the box and is not contained within it.
	return std::any_of(affecting_points.begin(), affecting_points.end(), [&](const auto &point) {
		return fullyOccludesBox(box, point, eye_center);
	});
}

/**
 * Check whether the ray that is occluded by this point crosses through a given bounding box.
 *
 * That is, the ray originating from the point and pointing away from the eye center must intersect the box.
 *
 * @param box 				The bounding box to check.
 * @param point 			The occluding point.
 * @param eye_center 		The eye point.
 * @return 					True if the ray is occluded by the point and crosses through the box, false otherwise.
 */
bool occludingRayCrossesCell(const Eigen::AlignedBox3d &box,
							 const OccupancyMap::OccludingPoint &point,
							 const Eigen::Vector3d &eye_center) {
	return math_utils::intersects(box, math_utils::Ray3d(point.point, eye_center - point.point));
}

/**
 * Check if the given point occludes the given box.
 *
 * @param box 			The box to check for occlusion.
 * @param point 		The point to check for occlusion.
 * @param eye_center 	The center of the eye (origin of the occlusion ray).
 * @return 				True if the point fully occludes the box, false otherwise.
 */
bool fullyOccludesBox(const Eigen::AlignedBox3d &box, const OccupancyMap::OccludingPoint &point, const Eigen::Vector3d &eye_center) {
	return occludingRayCrossesCell(box, point, eye_center) && !box.contains(point.point);
}

bool box_may_intersect_view_pyramid_sides(const Eigen::AlignedBox3d &box,
										  const math_utils::ViewPyramidFaces &planes) {// Sample the points closest to the cell center on the view pyramid planes.
	Eigen::Vector3d closest_view_pyramid_point = planes.closest_point_on_any_plane(box.center());

	bool may_intersect_planes = (closest_view_pyramid_point - box.center()).norm() < box.sizes().norm() / 2;
	return may_intersect_planes;
}

std::vector<OccupancyMap::OccludingPoint> filterAffectingPoints(const Eigen::AlignedBox3d &box,
																const std::vector<OccupancyMap::OccludingPoint> &candidate_occluding_points,
																const Eigen::Isometry3d &eye_transform) {
	return candidate_occluding_points | ranges::views::filter([&](const auto &p) {
					return math_utils::intersects(box,
												  math_utils::Ray3d(eye_transform.translation(),
																	p.point - eye_transform.translation()));
				}) | ranges::to_vector;
}

bool checkPointsInside(const Eigen::AlignedBox3d &box, std::vector<OccupancyMap::OccludingPoint> &affecting_points) {
	return std::any_of(affecting_points.begin(), affecting_points.end(), [&box](const auto &point) {
			return box.contains(point.point);
		});
}

bool cellIsFullySeen(Cell &cell) {
	return cell.is_leaf() && cell.get_leaf().data.isUniform() && cell.get_leaf().data.get_uniform_cell().seen;
}

/**
 * @brief Recursive function to incorporate a point cloud into the octree.
 *
 * @param box The bounding box of the current cell.
 * @param cell The current cell to incorporate the point cloud into.
 * @param candidate_occluding_points Vector of points to be incorporated; to be interpreted as rays from the eye_center
 * @param maxDepth The maximum depth of the octree.
 * @param eye_tranform The transform of the eye used to view the point cloud.
 * @param planes The view pyramid planes.
 */
void incorporate_internal(const Eigen::AlignedBox3d &box,
						  Cell &cell,
						  const std::vector<OccupancyMap::OccludingPoint> &candidate_occluding_points,
						  const int maxDepth,
						  const Eigen::Isometry3d &eye_transform,
						  const math_utils::ViewPyramidFaces &planes) {

	// If the cell is fully-seen, we can stop here.
	if (cellIsFullySeen(cell)) {
		return;
	}

	// Also, go find the points whose eye ray passes through the cell at all. This is essentially pre-filtering step on the input.
	auto affecting_points = filterAffectingPoints(box, candidate_occluding_points, eye_transform);

	bool may_intersect_planes = box_may_intersect_view_pyramid_sides(box, planes);

	bool points_inside = checkPointsInside(box, affecting_points);

	bool should_split = (points_inside || may_intersect_planes) && maxDepth > 0;

	if (cell.is_leaf() && should_split) {
		cell.split_by_copy({});
	}

	if (cell.is_split()) {

		SplitCell &split_cell = cell.get_split();

		OctantIterator octant_iterator(box);

		for (Cell &child: *split_cell.children) {
			// Recurse.
			incorporate_internal(*(octant_iterator++), child, affecting_points, maxDepth - 1, eye_transform, planes);
		}

		// If all children are identical, merge them.
		if (split_cell.has_uniform_leaves()) {
			cell.merge_cell();
		}

	} else {

		auto &leaf = cell.get_leaf();

		// Check if any of the points fully occlude this cell. If so, leave untouched.
		// A point fully occludes the cell if the ray cast from the point away from the eye center
		// intersects the cell, without the point itself being inside the cell.
		if (checkPointsFullyOccludingCell(box, affecting_points, eye_transform.translation()) || planes.contains(box.center()) && !may_intersect_planes) {
			return;
		}

		// If the cell cannot intersect the planes, and contains no occluding points, then it is fully visible.
		if (!may_intersect_planes && !points_inside) {
			leaf.data = {HierarchicalBoundaryCellAnnotatedRegionOctree::UniformCell{.seen = true}};
			return;
		}

		updateLeafData(box,
					   leaf.data,
					   plane_from_sample(box,
										 planes.closest_point_on_any_plane(box.center()),
										 eye_transform.translation()),
					   HierarchicalBoundaryCellAnnotatedRegionOctree::VIEW_PYRAMID_PLANE);

		for (const auto &point: affecting_points) {
			// If the point is outside the cell, ignore it.
			if (!box.contains(point.point)) {
				continue;
			}

			updateLeafData(box,
						   leaf.data,
						   plane_from_sample(box, point.point, eye_transform.translation()),
						   point.hard ? HierarchicalBoundaryCellAnnotatedRegionOctree::HARD_OBSTACLE
									  : HierarchicalBoundaryCellAnnotatedRegionOctree::SOFT_OBSTACLE);

		}

	}

}

void
HierarchicalBoundaryCellAnnotatedRegionOctree::incorporate(const std::vector<OccupancyMap::OccludingPoint> &occluding_points,
														   const Eigen::Isometry3d &eye_transform,
														   double fovX,
														   double fovY) {

	auto planes = math_utils::compute_view_pyramid_planes(eye_transform, fovX, fovY);

	Eigen::Vector3d in_front = eye_transform * Eigen::Vector3d(0, 1, 0);
	assert(planes.contains(in_front));

	incorporate_internal(tree.box, tree.root, occluding_points, max_depth, eye_transform, planes);

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

void HierarchicalBoundaryCellAnnotatedRegionOctree::LeafData::setBoundaryCell(const Plane3d &plane,
																			  HierarchicalBoundaryCellAnnotatedRegionOctree::BoundaryType boundaryType) {
	data = BoundaryCell{plane, boundaryType};
}
