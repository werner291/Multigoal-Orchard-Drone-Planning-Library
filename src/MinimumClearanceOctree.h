// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 6-12-22.
//

#ifndef NEW_PLANNERS_MINIMUMCLEARANCEOCTREE_H
#define NEW_PLANNERS_MINIMUMCLEARANCEOCTREE_H


#include <Eigen/Core>
#include <memory>
#include <optional>
#include <Eigen/Geometry>
#include "utilities/EigenExt.h"

/**
 *
 * An octree storing a minimum distance field, and a gradient thereof,
 * for a volume defined by an AABB.
 *
 * The volume is partitioned into a "clear" and "occupied" region; positive distance values are in the clear region,
 * negative distance values are in the occupied region.
 *
 * Unless a node is already at minimum size, and contains both a clear and an occupied point, the node is split into
 * eight children, and the node update computation is applied to each child recursively.
 *
 * The gradient is computed by taking the gradient of the distance field at the center of the node;
 * note that the gradient is not used to determine whether a node is split or not.
 *
 */
class MinimumClearanceOctree {

	// The bounds of the volume in which this MDF is defined
	Eigen::AlignedBox3d bounds;

	// The minimum size of leaf nodes. Nodes smaller than this will not be split.
	double min_leaf_size;

public:

	struct Node {
		double minimum_distance_at_center = -std::numeric_limits<double>::infinity();
		Eigen::Vector3d gradient_at_center = Eigen::Vector3d::Zero();
		std::unique_ptr<std::array<Node, 8> > children;
	};

private:

	// The root node of the octree, which contains the entire volume
	Node root;

public:

	[[nodiscard]] const Eigen::AlignedBox3d &getBounds() const;

	[[nodiscard]] double getMinLeafSize() const;

	[[nodiscard]] const Node &getRoot() const;

	MinimumClearanceOctree(const Eigen::AlignedBox3d &bounds, double minLeafSize);

	void
	update(const std::function<std::pair<double, Eigen::Vector3d>(const Eigen::Vector3d &)> &signedDistanceFunction);

	[[nodiscard]] std::pair<double, Eigen::Vector3d> getSignedDistanceAndGradient(const Eigen::Vector3d &point) const;

private:
	void
	updateInternal(const std::function<std::pair<double, Eigen::Vector3d>(const Eigen::Vector3d &)> &signedDistanceFunction,
				   Node &node,
				   const Eigen::AlignedBox3d &node_bounds);

};

/**
 * Extract the known "furthest closest" points from a MinimumClearanceOctree.
 *
 * @param octree 		The octree to extract the points from
 * @return 				A vector of points, each of which is the furthest closest point in the octree
 */
std::vector<Eigen::Vector3d> implicitSurfacePoints(const MinimumClearanceOctree &octree);


#endif //NEW_PLANNERS_MINIMUMCLEARANCEOCTREE_H
