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
	 * @param eye_transform The center of the eye used to view the point cloud.
	 * @param pointCloud The segmented point cloud to incorporate into the octree.
	 *
	 * Adds the points in the given point cloud to the octree, updating the region data as necessary.
	 */
	void incorporate(const std::vector<OccupancyMap::OccludingPoint> &occluding_points,
					 const Eigen::Isometry3d &eye_transform,
					 double fovX,
					 double fovY) override;

	/**
	 * A cell in the octree that is either fully seen or unseen.
	 */
	struct UniformCell {
		bool seen;
	};

	/**
	 * A cell in the octree that is partially seen, containing a linear approximation of the boundary
	 * of the observed region.
	 */
	struct BoundaryCell {
		// Linear approximation of the boundary of the observed region.
		// By convention, the normal points into the seen region.
		EigenExt::Plane3d plane;
		bool hard;
	};

	struct LeafData {
		std::variant<UniformCell, BoundaryCell> data;

		UniformCell &get_uniform_cell() {
			return std::get<UniformCell>(data);
		}

		const UniformCell &get_uniform_cell() const {
			return std::get<UniformCell>(data);
		}

		BoundaryCell &get_boundary_cell() {
			return std::get<BoundaryCell>(data);
		}

		const BoundaryCell &get_boundary_cell() const {
			return std::get<BoundaryCell>(data);
		}

		void setFullySeen();

		bool isUniform() const;

		bool isBoundary() const;

		void updateBoundary(const EigenExt::Plane3d &plane, bool hard, const Eigen::Vector3d &cell_center);
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
