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

class HierarchicalBoundaryCellAnnotatedRegionOctree : public OccupancyMap {
public:
	void incorporate(const Eigen::Vector3d &eye_center, const SegmentedPointCloud &pointCloud) override;

	void incorporate(const Eigen::Vector3d &eye_center, const RegionDefinitionFn &region_fn) override;

	RegionType query_at(const Eigen::Vector3d &query_point) override;

private:
	/**
	 * A leaf cell in the octree, labeled with a region,
	 * and optionally a plane that linearly approximates
	 * the boundary region where the labeled region starts.
	 *
	 * By convention, the normal points AWAY from the labeled region.
	 */
	struct LeafCell {
		RegionType region = UNSEEN;
		std::optional<EigenExt::Plane3d> plane{};
	};

	// Forward declaration of SplitCell since we use it in the Cell variant.
	struct SplitCell;

	// Cell, variant of either an internal cell (with children) or a leaf cell.
	using Cell = std::variant<SplitCell, LeafCell>;

	// A cell with 8 children. Children cover an octant of the parent,
	// in the order defined by OctantIterator in math_utils.h
	struct SplitCell {
		std::unique_ptr<std::array<Cell, 8> > children;
	};

	// The root cell. Initially, the whole space is UNSEEN.
	Cell root = {LeafCell{UNSEEN, std::nullopt}};

	// Center of the root bounding box.
	const Eigen::Vector3d center;
	// Edge length of the root bounding box (assumed cubical)
	const double base_edge_length;


public:
	HierarchicalBoundaryCellAnnotatedRegionOctree(const Eigen::Vector3d &center, double baseEdgeLength);
};


#endif //NEW_PLANNERS_HIERARCHICALBOUNDARYCELLANNOTATEDREGIONOCTREE_H
