// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-12-22.
//

#ifndef NEW_PLANNERS_HIERARCHICALCATEGORICALOCCUPANCYOCTREE_H
#define NEW_PLANNERS_HIERARCHICALCATEGORICALOCCUPANCYOCTREE_H

#include "Octree.h"
#include "OccupancyMap.h"
#include "VisibilityBoundary.h"

/**
 * HierarchicalCategoricalOccupancyOctree is a simple octree data structure that stores categorical data at its leaf nodes.
 * The leaf nodes can be labeled as either "unseen", "free", or "occupied".
 * This class extends the OccupancyMap class and implements the incorporate and query_at functions. The incorporate function
 * allows the octree to incorporate new data points into the tree, while the query_at function allows the user to query the
 * octree for the label of a specific point in 3D space.
 * The internal data structure of the octree is an Octree template class, with the template parameter RegionType representing
 * the categorical data at the leaf nodes.
 */
class HierarchicalCategoricalOccupancyOctree : public OccupancyMap {

public:
	enum RegionType {
		UNSEEN,
		FREE,
		OCCUPIED
	};

	/**
     * @fn void incorporate(const Eigen::Vector3d &eye_center, const SegmentedPointCloud &pointCloud)
     * Incorporates a segmented point cloud into the octree, using the given eye center as a reference point.
     * @param eye_transform
     * The center of the robot's sensor, used as a reference point for incorporating the point cloud.
     * @param pointCloud
     * The segmented point cloud to be incorporated into the octree.
     */
	void incorporate(const std::vector<OccupancyMap::OccludingPoint> &occluding_points,
					 const Eigen::Isometry3d &eye_transform,
					 double fovX,
					 double fovY) override;

	/**
	 * @using CategoricalOctree A typedef for an octree that stores categorical data (either UNSEEN, FREE, or OCCUPIED)
	 * at its leaf nodes. This octree is used as the underlying data structure for HierarchicalCategoricalOccupancyOctree.
     * @see HierarchicalCategoricalOccupancyOctree, Octree, RegionType
    */
	using CategoricalOctree = Octree<std::monostate, RegionType>;

	/**
     * @var CategoricalOctree tree The underlying octree data structure for the HierarchicalCategoricalOccupancyOctree class. This octree stores categorical data (either UNSEEN, FREE, or OCCUPIED) at its leaf nodes.
     * @see HierarchicalCategoricalOccupancyOctree, CategoricalOctree, RegionType
     */
	CategoricalOctree tree{RegionType::UNSEEN};

	/**
	 * Constructor.
	 *
	 * @param center The center of the octree.
	 * @param baseEdgeLength The base edge length of the octree.
	 * @param maxDepth The maximum depth of the octree.
	 *
	 * This will make the octree cover a cube with the given center and base edge length.
	 *
	 * The smallest cell will have edge length baseEdgeLength / 2^maxDepth.
	 */
	HierarchicalCategoricalOccupancyOctree(const Eigen::Vector3d &center,
										   double baseEdgeLength,
										   int maxDepth);

	/// The maximum depth of the octree. The smallest cell will have edge length baseEdgeLength / 2^maxDepth.
	const int maxDepth;

	/**
	 * Return the minimum edge length of the octree.
	 */
	double getMinEdgeLength() const;

	/**
	 * Return the underlying octree.
	 */
	[[nodiscard]] const CategoricalOctree &getOctree() const;

	[[nodiscard]] RegionType query_at(const Eigen::Vector3d &query_point) const;
};

#endif //NEW_PLANNERS_HIERARCHICALCATEGORICALOCCUPANCYOCTREE_H
