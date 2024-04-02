// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/21/23.
//

#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/math/triangle.h>

#include "fcl_utils.h"

using namespace mgodpl;
using namespace fcl;

namespace mgodpl::fcl_utils {

	std::shared_ptr<fcl::BVHModel<fcl::OBBd> > meshToFclBVH(const mgodpl::Mesh &shape) {

		// Create a shared pointer to the BVHModel
		auto g = std::make_shared<fcl::BVHModel<fcl::OBBd>>();

		// Create a vector to store FCL triangles
		std::vector<fcl::Triangle> tri_indices(shape.triangles.size());
		for (unsigned int i = 0; i < shape.triangles.size(); ++i) {
			// Populate FCL Triangle indices from the Mesh message
			tri_indices[i] = fcl::Triangle(
					shape.triangles[i][0],
					shape.triangles[i][1],
					shape.triangles[i][2]
			);
		}

		// Create a vector to store FCL Vector3d points
		std::vector<fcl::Vector3d> points(shape.vertices.size());
		for (unsigned int i = 0; i < shape.vertices.size(); ++i) {
			// Populate FCL Vector3d points from the Mesh message
			points[i] = fcl::Vector3d(shape.vertices[i].x(), shape.vertices[i].y(), shape.vertices[i].z());
		}

		// Begin the construction of the BVHModel
		g->beginModel();

		// Add the sub-model to the BVHModel using the points and triangles
		g->addSubModel(points, tri_indices);

		// End the construction of the BVHModel
		g->endModel();

		// Return the constructed BVHModel as a shared pointer
		return g;
	}
}