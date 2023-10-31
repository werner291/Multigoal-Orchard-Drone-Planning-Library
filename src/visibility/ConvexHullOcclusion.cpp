// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/31/23.
//

#include "ConvexHullOcclusion.h"
#include "convex_hull.h"

void mgodpl::visibility::ConvexOcclusion::reveal(const mgodpl::math::Vec3d &eye) {

	for (int i = 0; i < _mesh.faces.size(); ++i) {

		if (seen_faces[i]) { // Skip faces we have already seen.
			continue;
		}

		const auto& triangle = _mesh.triangle(i);

		// Get the normal of the triangle.
		const math::Vec3d normal = triangle.normal();

		// If the normal faces away from the eye, we keep the triangle.
		if (normal.dot(triangle.a - eye) > 0) {
			seen_faces[i] = true;
		}
	}

}

mgodpl::visibility::ConvexOcclusion
mgodpl::visibility::ConvexOcclusion::from_points(const std::vector<math::Vec3d> &points) {

	auto chull = convexHull(points);

	return {
		._mesh = chull,
		.seen_faces = std::vector<bool>(chull.faces.size(), false)
	};
}
