// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/18/24.
//

#include "Mesh.h"

mgodpl::math::AABBd mgodpl::mesh_aabb(const Mesh &mesh) {
	math::AABBd aabb = math::AABBd::inverted_infinity();
	for (const auto &vertex: mesh.vertices) {
		aabb.expand(vertex);
	}
	return aabb;
}

std::vector<std::array<mgodpl::math::Vec3d, 3>> mgodpl::triangles_from_mesh(const Mesh &mesh) {
	std::vector<std::array<math::Vec3d,3>> triangles;
	for (size_t i = 0; i < mesh.triangles.size(); i += 3) {
		triangles.push_back({
									mesh.vertices[mesh.triangles[i][0]],
									mesh.vertices[mesh.triangles[i][1]],
									mesh.vertices[mesh.triangles[i][2]]
							});
	}
	return triangles;
}
