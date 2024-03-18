// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/18/24.
//

#ifndef MGODPL_MESH_H
#define MGODPL_MESH_H

#include <vector>
#include <array>
#include "../math/Vec3.h"
#include "../math/AABB.h"

namespace mgodpl {

	struct Mesh {
		std::vector<mgodpl::math::Vec3d> vertices;
		std::vector<std::array<size_t, 3>> triangles;
	};

	/**
	 * Computes the AABB of the mesh.
	 */
	math::AABBd mesh_aabb(const Mesh &mesh);

	std::vector<std::array<math::Vec3d,3>> triangles_from_mesh(const Mesh &mesh);
}

#endif //MGODPL_MESH_H
