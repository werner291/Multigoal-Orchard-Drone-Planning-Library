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

class HierarchicalBoundaryCellAnnotatedRegionOctree : public OccupancyMap {
public:
	void incorporate(const Eigen::Vector3d &eye_center, const SegmentedPointCloud &pointCloud) override;

	void incorporate(const Eigen::Vector3d &eye_center, const RegionDefinitionFn &region_fn) override;

	RegionType query_at(const Eigen::Vector3d &query_point) override;

	/**
	 * A leaf cell in the octree, labeled with a region,
	 * and optionally a plane that linearly approximates
	 * the boundary region where the labeled region starts.
	 *
	 * By convention, the normal points AWAY from the labeled region.
	 */
	struct LeafData {
		RegionType region = UNSEEN;
		std::optional<EigenExt::Plane3d> plane{};

		bool operator==(const LeafData &other) const {
			return region == other.region && plane->coeffs() == other.plane->coeffs();
		}
	};

	using PointAnnotatedOctree = Octree<std::monostate, LeafData>;

	PointAnnotatedOctree tree;

	HierarchicalBoundaryCellAnnotatedRegionOctree(const Eigen::Vector3d &center, double baseEdgeLength, const unsigned int maxDepth);

	const unsigned int max_depth;
};


#endif //NEW_PLANNERS_HIERARCHICALBOUNDARYCELLANNOTATEDREGIONOCTREE_H
