// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "MinimumClearanceOctree.h"

void MinimumClearanceOctree::clearSphereInternal(const CenterRadiusSphere &sphere,
												 MinimumClearanceOctree::Node &node,
												 const Eigen::AlignedBox3d &node_bounds) {

	Eigen::Vector3d cell_center = node_bounds.center();

	double cell_center_signed_distance = node.plane.signedDistance(cell_center);

	Eigen::Vector3d sphere_delta = cell_center - sphere.center;

	Eigen::Vector3d normal = sphere_delta.normalized();
	Eigen::Vector3d tangent_point = sphere.center - normal * sphere.radius;

}

void MinimumClearanceOctree::declareClearSphere(const CenterRadiusSphere &sphere) {

	clearSphereInternal(sphere, root, bounds);

}
