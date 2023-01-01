// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-12-22.
//

#ifndef NEW_PLANNERS_HIERARCHICALBOUNDARYCELLANNOTATEDREGIONOCTREE_H
#define NEW_PLANNERS_HIERARCHICALBOUNDARYCELLANNOTATEDREGIONOCTREE_H

#include <Eigen/Core>
#include <memory>
#include <optional>
#include <variant>
#include "../utilities/EigenExt.h"
#include "OccupancyMap.h"
#include "Octree.h"

/**
 * An octree occupancy map that can be explicitly annotated with a linear approximation of the
 * boundary of the occupied region in each of the leaf cells.
 */
class HierarchicalBoundaryCellAnnotatedRegionOctree : public OccupancyMap {

public:

	/**
	 * @brief Incorporate a segmented point cloud into the octree.
	 *
	 * @param eye_center The center of the eye used to view the point cloud.
	 * @param pointCloud The segmented point cloud to incorporate into the octree.
	 *
	 * Adds the points in the given point cloud to the octree, updating the region data as necessary.
	 */
	void incorporate(const Eigen::Vector3d &eye_center, const SegmentedPointCloud &pointCloud) override;

	/**
	 * @brief Incorporate a region defined by a function into the octree.
	 *
	 * @param eye_center The center of the eye used to view the region.
	 * @param region_fn A function that returns true for points that are within the region, false otherwise.
	 *
	 * Adds the points in the region defined by the given function to the octree, updating the region data as necessary.
	 */
	void incorporate(const Eigen::Vector3d &eye_center, const RegionDefinitionFn &region_fn) override;

	/**
	 * @brief Query the region at the given point in the octree.
	 *
	 * @param query_point The point to query the region for.
	 *
	 * @return The region at the given point in the octree.
	 *
	 * Causes assertion failure if the query point is not inside the tree's bounding box.
	 */
	RegionType query_at(const Eigen::Vector3d &query_point) const override;

	/**
	 * A leaf cell in the octree, labeled with a region, and optionally a plane that linearly approximates
	 * the boundary region where the labeled region starts.
	 *
	 * By convention, the normal points AWAY from the labeled region.
	 */
	struct LeafData {

		/// The region type of the cell.
		RegionType region = UNSEEN;

		/**
		 * The plane that linearly approximates the boundary of the labeled region.
		 * If present, the cell is to be considered partially filled with the labeled region,
		 * delimited by the plane; the normal points AWAY from the labeled region.
		 */
		std::optional<EigenExt::Plane3d> plane{};

		/**
		 * Equality operator for LeafData, true if the region and plane are exactly equal (beware of floating point errors!).
		 */
		bool operator==(const LeafData &other) const {
			return region == other.region && plane->coeffs() == other.plane->coeffs();
		}
	};

	/// The Octree type; SplitCells are not annotated (monostate), LeafCells are annotated with a LeafData.
	using PointAnnotatedOctree = Octree<std::monostate, LeafData>;

	/// The underlying octree.
	PointAnnotatedOctree tree;

	/**
	 * @brief Get the underlying octree.
	 */
	[[nodiscard]] const PointAnnotatedOctree &getTree() const;

	/**
	 * Get the maximum depth of the octree; the smallest leaf cells are baseEdgeLength / 2^maxDepth in size.
	 * @return 		The maximum depth of the octree.
	 */
	[[nodiscard]] const unsigned int getMaxDepth() const;


	/**
	 * @brief Construct a HierarchicalBoundaryCellAnnotatedRegionOctree with the given center, base edge length, and maximum depth.
	 * @param center The center point of the octree.
	 * @param baseEdgeLength The length of the edges of the root cell.
	 * @param maxDepth The maximum depth of the octree.
	 */
	HierarchicalBoundaryCellAnnotatedRegionOctree(const Eigen::Vector3d &center,
												  double baseEdgeLength,
												  const unsigned int maxDepth);

	/// Maximum depth of the octree.
	const unsigned int max_depth;


};


#endif //NEW_PLANNERS_HIERARCHICALBOUNDARYCELLANNOTATEDREGIONOCTREE_H
