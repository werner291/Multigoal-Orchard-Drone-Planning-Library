// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 4/8/24.
//

#include "plan_path_from_parameters.h"
#include "../planning/state_tools.h"
#include "../planning/probing_motions.h"

using namespace mgodpl;
using namespace mgodpl::declarative;

std::vector<math::Vec3d> fruit_positions_from_models(const FruitModels &fruit_models) {
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

mgodpl::RobotPath mgodpl::plan_multigoal_path_in_scenario(const SolutionMethod &method,
														  const PointScanEnvironment &env) {
	if (auto facing_tree = std::get_if<OrbitFacingTree>(&method)) {
		return parametricPathToRobotPath(env.robot,
										 env.tree_model->leaves_aabb.center(),
										 instantiatePath(
												 facing_tree->params, env.tree_model->leaves_aabb.center(),
												 env.tree_model->canopy_radius), 1000);
	} else if (std::get_if<ProbingMotionsMethod>(&method)) {

		const auto fruit_positions = fruit_positions_from_models(env.fruit_models);

		return plan_multigoal_path(env.robot,
								   env.tree_model->meshes.trunk_mesh,
								   env.tree_model->meshes.leaves_mesh,
								   fruit_positions,
								   fromEndEffectorAndVector(env.robot, {0, 5, 5}, {0, 1, 1}));
	}
}
