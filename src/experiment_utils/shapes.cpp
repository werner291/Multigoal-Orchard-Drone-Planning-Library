// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/23/23.
//

#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include "shapes.h"
#include "load_mesh_ros.h"

mgodpl::Mesh mgodpl::loadMesh(const std::string &path) {
	const auto& msg_mesh = loadRobotMesh("test_robots/meshes/drone.dae");

	return Mesh {
			msg_mesh.vertices | ranges::views::transform([](const auto& vertex) {
				return math::Vec3d {
						vertex.x,
						vertex.y,
						vertex.z
				};
			}) | ranges::to<std::vector>(),
			msg_mesh.triangles | ranges::views::transform([](const auto& triangle) {
				return std::array<size_t, 3> {
						triangle.vertex_indices[0],
						triangle.vertex_indices[1],
						triangle.vertex_indices[2]
				};
			}) | ranges::to<std::vector>()
	};
}
