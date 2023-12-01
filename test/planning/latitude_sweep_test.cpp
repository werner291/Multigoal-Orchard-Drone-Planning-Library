// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 12/1/23.
//

#include <gtest/gtest.h>
#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>

#include "../../src/planning/latitude_sweep.h"
#include "../../src/experiment_utils/TreeMeshes.h"

using namespace mgodpl;

TEST(latitude_sweep_tests, kinetic_datastructure_vs_bruteforce) {

	// Get a tree model.
	const auto& model = mgodpl::tree_meshes::loadTreeMeshes("appletree");

	const math::Vec3d target = {0.2, 0.5, 3.0};

	const std::vector<Triangle> triangles = model.trunk_mesh.triangles | ranges::views::transform(
			[&](const auto& triangle)
			{
				return Triangle{
						{
								math::Vec3d(model.trunk_mesh.vertices[triangle.vertex_indices[0]].x,
											model.trunk_mesh.vertices[triangle.vertex_indices[0]].y,
											model.trunk_mesh.vertices[triangle.vertex_indices[0]].z),
								math::Vec3d(model.trunk_mesh.vertices[triangle.vertex_indices[1]].x,
											model.trunk_mesh.vertices[triangle.vertex_indices[1]].y,
											model.trunk_mesh.vertices[triangle.vertex_indices[1]].z),
								math::Vec3d(model.trunk_mesh.vertices[triangle.vertex_indices[2]].x,
											model.trunk_mesh.vertices[triangle.vertex_indices[2]].y,
											model.trunk_mesh.vertices[triangle.vertex_indices[2]].z)
						}
				};
			}) | ranges::to<std::vector>();

	double longitude = 0.0;

	while (longitude < 2.0 * M_PI + 0.1) {
		{

			longitude += 0.0005;

			std::vector<math::Vec3d> points;
			for (int lat_i = 0; lat_i <= 32; ++lat_i) {
				// Latitude from [-pi/2, pi/2]
				double latitude = -M_PI / 2.0 + lat_i * M_PI / 32.0;

				math::Vec3d ray(
						cos(latitude) * cos(longitude),
						cos(latitude) * sin(longitude),
						sin(latitude)
				);

				points.push_back(target + ray * 1.0);
			}

			const auto &intersections = triangle_intersections(
					triangles,
					target,
					longitude
			);

			std::vector<std::array<math::Vec3d, 3>> intersection_points;
			for (const auto &latitudes: free_latitude_ranges(intersections, target, longitude, triangles, 0.0)) {
				double length = latitudes[1] - latitudes[0];

				size_t n_points = std::max(1, (int) (length * 16.0));

				for (int lat_i = 0; lat_i < n_points; ++lat_i) {
					double latitude1 = latitudes[0] + lat_i * (latitudes[1] - latitudes[0]) / (double) n_points;
					double latitude2 = latitudes[0] + (lat_i + 1) * (latitudes[1] - latitudes[0]) / (double) n_points;

					math::Vec3d ray1(
							cos(latitude1) * cos(longitude),
							cos(latitude1) * sin(longitude),
							sin(latitude1)
					);

					math::Vec3d ray2(
							cos(latitude2) * cos(longitude),
							cos(latitude2) * sin(longitude),
							sin(latitude2)
					);

					std::array<math::Vec3d, 3> triangle_points{
							target + ray1 * 1.0,
							target + ray2 * 1.0,
							target
					};

					intersection_points.push_back(triangle_points);
				}
			}
		}

}