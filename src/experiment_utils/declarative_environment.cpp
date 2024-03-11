// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/11/24.
//

#include <random>
#include "declarative_environment.h"
#include "leaf_scaling.h"
#include "mesh_utils.h"
#include "procedural_fruit_placement.h"

mgodpl::declarative::LoadedTreeModel mgodpl::declarative::LoadedTreeModel::from_name(const std::string &name) {
	const auto meshes = mgodpl::tree_meshes::loadTreeMeshes(name);
	const auto root_points = leaf_root_points(meshes);
	const auto leaves_aabb = mesh_aabb(meshes.leaves_mesh);
	const auto canopy_radius = std::min(leaves_aabb.size().x(), leaves_aabb.size().y()) / 2.0;

	return {
			.meshes = meshes,
			.root_points = root_points,
			.leaves_aabb = leaves_aabb,
			.canopy_radius = canopy_radius
	};
}

Json::Value mgodpl::declarative::toJson(const mgodpl::declarative::PointScanEvalParameters &params) {
	Json::Value json;
	json["tree_params"] = toJson(params.tree_params);
	json["sensor_params"] = toJson(params.sensor_params);
	json["n_scannable_points_per_fruit"] = params.n_scannable_points_per_fruit;
	return json;
}

std::vector<shape_msgs::msg::Mesh>
mgodpl::declarative::random_subset_of_meshes(random_numbers::RandomNumberGenerator &rng,
											 const std::shared_ptr<const LoadedTreeModel> &tree_model,
											 const RandomSubset &subset_params) {
	std::vector<shape_msgs::msg::Mesh> fruit_meshes = tree_model->meshes.fruit_meshes;

	if (subset_params.count > fruit_meshes.size()) {
		throw std::runtime_error("Requested more fruit than available in the tree model");
	}

	// Using rng to shuffle the fruit meshes; we draw the seed from the rng to ensure reproducibility.
	std::mt19937 stdrng(rng.uniformInteger(0, std::numeric_limits<int>::max()));

	std::sample(fruit_meshes.begin(), fruit_meshes.end(), std::back_inserter(fruit_meshes), subset_params.count, stdrng);
	return fruit_meshes;
}

std::shared_ptr<const mgodpl::declarative::LoadedTreeModel>
mgodpl::declarative::EnvironmentInstanceCache::obtain_tree_model(const std::string &tree_name) {

	// Lock the mutex for accessing the cache.
	std::lock_guard<std::mutex> lock(mutex);

	// Add a new tree model to the cache if it does not exist.
	if (tree_models.find(tree_name) == tree_models.end()) {
		tree_models.insert({tree_name, std::make_shared<LoadedTreeModel>(LoadedTreeModel::from_name(tree_name))});
	}

	// Return the tree model from the cache.
	return tree_models.find(tree_name)->second;
}

mgodpl::declarative::PointScanEnvironment
mgodpl::declarative::EnvironmentInstanceCache::create_environment(const mgodpl::declarative::PointScanEvalParameters &params) {

	random_numbers::RandomNumberGenerator rng(params.tree_params.seed);

	// Load the tree model
	std::shared_ptr<const LoadedTreeModel> tree_model = obtain_tree_model(params.tree_params.name);

	// Create the scannable points
	std::vector<std::vector<SurfacePoint>> all_scannable_points;

	if (std::holds_alternative<Unchanged>(params.tree_params.fruit_subset)) {
		for (const auto &fruit_mesh: tree_model->meshes.fruit_meshes) {
			all_scannable_points.push_back(sample_points_on_mesh(rng, fruit_mesh, params.n_scannable_points_per_fruit));
		}
	} else if (const auto& subset_params = std::get_if<RandomSubset>(&params.tree_params.fruit_subset)) {

		// Pick a random subset without replacement of the fruit meshes.
		for (const auto &fruit_mesh: random_subset_of_meshes(rng, tree_model, *subset_params)) {
			all_scannable_points.push_back(sample_points_on_mesh(rng, fruit_mesh, params.n_scannable_points_per_fruit));
		}

	} else if (const auto& fruit_subset = std::get_if<Replace>(&params.tree_params.fruit_subset)) {
		auto fruit_locations = generate_fruit_locations(tree_model->meshes, fruit_subset->count, rng);

		all_scannable_points.reserve(fruit_locations.size());

		const double FRUIT_RADIUS = 0.05;

		std::vector<SurfacePoint> fruit_points;

		for (const auto &fruit_location: fruit_locations) {
			for (size_t i = 0; i < params.n_scannable_points_per_fruit; ++i)
			{
				math::Vec3d random_direction(rng.gaussian01(), rng.gaussian01(), rng.gaussian01());
				random_direction.normalize();
				math::Vec3d random_position = fruit_location + random_direction * FRUIT_RADIUS;
				fruit_points.push_back({random_position, random_direction});
			}
		}

		all_scannable_points.push_back(fruit_points);
	}

	// Scale the leaves
	const auto scaled_leaves = scale_leaves(tree_model->meshes,
											tree_model->root_points,
											params.tree_params.leaf_scale);

	// Return the environment instance
	return {
			.robot = robot,
			.tree_model = tree_model,
			.scaled_leaves = scaled_leaves,
			.scannable_points = all_scannable_points,
			.mesh_occlusion_model = std::make_shared<MeshOcclusionModel>(scaled_leaves, 0.0)
	};
}
