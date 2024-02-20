// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 2/20/24.
//

#include "procedural_fruit_placement.h"
#include "surface_points.h"

using namespace mgodpl;

std::vector<math::Vec3d> mgodpl::generate_fruit_locations(const mgodpl::tree_meshes::TreeMeshes &tree_model,
														  size_t num_fruits,
														  random_numbers::RandomNumberGenerator &rng) {
	std::vector<math::Vec3d> fruit_locations;

	const auto &cumulative_areas = triangle_cumulative_areas(tree_model.trunk_mesh);

	while (fruit_locations.size() < num_fruits) {

		auto pt = sample_point_on_mesh(rng, tree_model.trunk_mesh, cumulative_areas);

		// If within 0.5 units of the vertical axis and below 1.5 units (likely on the trunk), skip.
		if (pt.position.x() * pt.position.x() + pt.position.y() * pt.position.y() < 0.25 && pt.position.z() < 1.5) {
			continue;
		}

		fruit_locations.push_back(pt.position + pt.normal * FRUIT_RADIUS);
	}

	return fruit_locations;
}

std::vector<math::Vec3d> mgodpl::generate_fruit_clusters(const tree_meshes::TreeMeshes &tree_model,
												 size_t num_clusters,
												 size_t min_cluster_size,
												 size_t max_cluster_size,
												 bool bias_towards_small_clusters,
												 random_numbers::RandomNumberGenerator &rng) {

	std::vector<math::Vec3d> fruit_locations;

	const auto &cumulative_areas = triangle_cumulative_areas(tree_model.trunk_mesh);

	while (fruit_locations.size() < num_clusters) {

		auto pt = sample_point_on_mesh(rng, tree_model.trunk_mesh, cumulative_areas);

		// If within 0.5 units of the vertical axis and below 1.5 units (likely on the trunk), skip.
		if (pt.position.x() * pt.position.x() + pt.position.y() * pt.position.y() < 0.25 && pt.position.z() < 1.5) {
			continue;
		} else {
			num_clusters++;
		}

		size_t cluster_size = rng.uniformInteger(min_cluster_size, max_cluster_size);

		if (bias_towards_small_clusters) {
			// Do it again, take the smaller.
			cluster_size = std::min(cluster_size, (size_t) rng.uniformInteger(min_cluster_size, max_cluster_size));
		}

		fruit_locations.push_back(pt.position + pt.normal * FRUIT_RADIUS);

		for (size_t i = 1; i < cluster_size; i++) {
			// Generate a random vector in the plane perpendicular to the normal
			math::Vec3d random_vector(rng.gaussian01(), rng.gaussian01(), rng.gaussian01());
			random_vector = (random_vector - pt.normal * random_vector.dot(pt.normal)).normalized();
			fruit_locations.push_back(pt.position + random_vector * FRUIT_RADIUS);
		}
	}

	return fruit_locations;

}
