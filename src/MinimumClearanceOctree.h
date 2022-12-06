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

struct CenterRadiusSphere {
	Eigen::Vector3d center;
	double radius;
};

class MinimumClearanceOctree {

	Eigen::AlignedBox3d bounds;
	double min_leaf_size;

public:
	struct Node {
		/// A linear approximation of the surface of the surface of the volume, at the point closest to the center of the node.
		EigenExt::Plane3d plane;
		std::unique_ptr<std::array<Node, 8> > children;
	};

private:


	Node root;

	void clearSphereInternal(const CenterRadiusSphere &sphere, Node &node, const Eigen::AlignedBox3d &node_bounds);

public:

	/**
	 * Declare that a given spherical volume is clear of any obstacles.
	 *
	 * @param center
	 * @param radius
	 */
	void declareClearSphere(const CenterRadiusSphere &sphere);

};


#endif //NEW_PLANNERS_MINIMUMCLEARANCEOCTREE_H
