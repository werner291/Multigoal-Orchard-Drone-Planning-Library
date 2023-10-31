// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/31/23.
//

#ifndef MGODPL_CONVEXHULLOCCLUSION_H
#define MGODPL_CONVEXHULLOCCLUSION_H

#include <vector>
#include "../math/Vec3.h"
#include "Mesh.h"

namespace mgodpl::visibility {

	/**
	 * Representative of a partially-revealed convex hull.
	 */
	struct ConvexOcclusion {

		// Internal mesh:
		Mesh _mesh;

		std::vector<bool> seen_faces;

		/**
		 * Set all faces whose normal points to the eye to true.
		 * @param eye
		 */
		void reveal(const math::Vec3d &eye);

		static ConvexOcclusion from_points(const std::vector<math::Vec3d> &points);
	};

}

#endif //MGODPL_CONVEXHULLOCCLUSION_H
