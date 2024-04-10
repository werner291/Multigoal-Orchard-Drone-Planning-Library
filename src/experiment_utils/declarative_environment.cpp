// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/11/24.
//

#include <random>
#include "declarative_environment.h"
#include "leaf_scaling.h"
#include "procedural_fruit_placement.h"
#include "procedural_robot_models.h"
#include "LoadedTreeModel.h"
#include "../planning/state_tools.h"

mgodpl::experiments::LoadedTreeModel mgodpl::experiments::LoadedTreeModel::from_name(const std::string &name) {
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

std::vector<mgodpl::Mesh>
mgodpl::declarative::random_subset_of_meshes(random_numbers::RandomNumberGenerator &rng,
											 const experiments::LoadedTreeModel& tree_model,
											 const RandomSubset &subset_params) {
	std::vector<Mesh> fruit_meshes = tree_model.meshes.fruit_meshes;

	if (subset_params.count > fruit_meshes.size()) {
		throw std::runtime_error("Requested more fruit than available in the tree model");
	}

	// Using rng to shuffle the fruit meshes; we draw the seed from the rng to ensure reproducibility.
	std::mt19937 stdrng(rng.uniformInteger(0, std::numeric_limits<int>::max()));

	std::sample(fruit_meshes.begin(), fruit_meshes.end(), std::back_inserter(fruit_meshes), subset_params.count, stdrng);
	return fruit_meshes;
}

mgodpl::declarative::FruitModels
mgodpl::declarative::instantiate_fruit_models(const mgodpl::experiments::LoadedTreeModel &tree_model,
											  const mgodpl::declarative::FruitSubset &tree_params,
											  random_numbers::RandomNumberGenerator &rng) {
	if (std::holds_alternative<Unchanged>(tree_params)) {
		std::vector<MeshFruit> fruit_meshes;
		for (const auto &fruit_mesh: tree_model.meshes.fruit_meshes) {
			fruit_meshes.push_back({fruit_mesh, mesh_aabb(fruit_mesh).center()});
		}
		return fruit_meshes;
	} else if (const auto& subset_params = std::get_if<RandomSubset>(&tree_params)) {
		auto meshes = random_subset_of_meshes(rng, tree_model, *subset_params);
		std::vector<MeshFruit> fruit_meshes;
		for (const auto &fruit_mesh: meshes) {
			fruit_meshes.push_back({fruit_mesh, mesh_aabb(fruit_mesh).center()});
		}
	} else if (const auto& replace_params = std::get_if<Replace>(&tree_params)) {
		auto fruit_locations = generate_fruit_locations(tree_model.meshes, replace_params->count, rng);
		std::vector<SphericalFruit> fruit_centers;
		fruit_centers.reserve(fruit_locations.size());
		for (const auto &fruit_location: fruit_locations) {
			fruit_centers.push_back({fruit_location, 0.25});
		}
		return fruit_centers;
	}

	throw std::runtime_error("Unknown fruit subset type");
}

mgodpl::declarative::FullTreeModel
mgodpl::declarative::instantiate_tree_model(const mgodpl::declarative::TreeModelParameters &tree_params,
											mgodpl::experiments::TreeModelCache &cache,
											random_numbers::RandomNumberGenerator &rng) {

	auto tree_model = cache.obtain_by_name(tree_params.name);

	auto fruit_models = instantiate_fruit_models(*tree_model, tree_params.fruit_subset, rng);

	auto scaled_leaves = scale_leaves(tree_model->meshes, tree_model->root_points, tree_params.leaf_scale);

	return FullTreeModel {
			.tree_model = tree_model,
			.fruit_models = fruit_models,
			.scaled_leaves = scaled_leaves
	};
}

std::vector<mgodpl::math::Vec3d>
mgodpl::declarative::fruit_positions_from_models(const mgodpl::declarative::FruitModels &fruit_models) {
	std::vector<math::Vec3d> fruit_positions;
	if (auto meshes = std::get_if<std::vector<MeshFruit>>(&fruit_models)) {
		for (const auto &mesh : *meshes) {
			fruit_positions.push_back(mesh.center);
		}
	} else if (auto spheres = std::get_if<std::vector<SphericalFruit>>(&fruit_models)) {
		for (const auto &sphere : *spheres) {
			fruit_positions.push_back(sphere.center);
		}
	} else {
		throw std::runtime_error("Unsupported fruit model type");
	}
	return fruit_positions;
}

std::vector<std::vector<mgodpl::SurfacePoint>> mgodpl::declarative::generate_scannable_points(
		const mgodpl::declarative::FruitModels &fruit_models,
		size_t n_points_per_fruit,
		random_numbers::RandomNumberGenerator &rng) {

	std::vector<std::vector<mgodpl::SurfacePoint>> all_scannable_points;

	if (const auto& mesh_models = std::get_if<std::vector<MeshFruit>>(&fruit_models)) {
		for (const auto &fruit_mesh: *mesh_models) {
			std::vector<SurfacePoint> fruit_points;
			fruit_points.reserve(n_points_per_fruit);
			for (size_t i = 0; i < n_points_per_fruit; ++i)
			{
				math::Vec3d random_direction(rng.gaussian01(), rng.gaussian01(), rng.gaussian01());
				random_direction.normalize();
				math::Vec3d random_position = fruit_mesh.center + random_direction * 0.05;
				fruit_points.push_back({random_position, random_direction});
			}
			all_scannable_points.push_back(fruit_points);
		}
	} else if (const auto& spherical_models = std::get_if<std::vector<SphericalFruit>>(&fruit_models)) {
		for (const auto &fruit_center: *spherical_models) {
			std::vector<SurfacePoint> fruit_points;
			fruit_points.reserve(n_points_per_fruit);
			for (size_t i = 0; i < n_points_per_fruit; ++i)
			{
				math::Vec3d random_direction(rng.gaussian01(), rng.gaussian01(), rng.gaussian01());
				random_direction.normalize();
				math::Vec3d random_position = fruit_center.center + random_direction * 0.05;
				fruit_points.push_back({random_position, random_direction});
			}
			all_scannable_points.push_back(fruit_points);
		}
	}

	return all_scannable_points;

}


mgodpl::declarative::PointScanEnvironment
mgodpl::declarative::create_environment(const mgodpl::declarative::PointScanEvalParameters &params, experiments::TreeModelCache &tree_model_cache) {

	random_numbers::RandomNumberGenerator rng(params.tree_params.seed);

	// Load the tree model
	std::shared_ptr<const experiments::LoadedTreeModel> tree_model = tree_model_cache.obtain_by_name(params.tree_params.name);

	// Create the scannable points
	const FruitModels fruit_models = instantiate_fruit_models(*tree_model, params.tree_params.fruit_subset, rng);

	const auto all_scannable_points = generate_scannable_points(fruit_models, params.n_scannable_points_per_fruit, rng);

	// Scale the leaves
	const auto scaled_leaves = scale_leaves(tree_model->meshes,
											tree_model->root_points,
											params.tree_params.leaf_scale);

	robot_model::RobotModel robot = experiments::createProceduralRobotModel();

	RobotState initial_state = fromEndEffectorAndVector(robot, {0, 5, 5}, {0, 1, 1});

	// Return the environment instance
	return {
			.robot = robot,
			.tree_model = tree_model,
			.scaled_leaves = scaled_leaves,
			.fruit_models = fruit_models,
			.scannable_points = all_scannable_points,
			.mesh_occlusion_model = std::make_shared<MeshOcclusionModel>(scaled_leaves, 0.0),
			.initial_state = initial_state
	};
}
