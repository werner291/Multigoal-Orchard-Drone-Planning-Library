// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/11/24.
//

#ifndef MGODPL_DECLARATIVE_ENVIRONMENT_H
#define MGODPL_DECLARATIVE_ENVIRONMENT_H

#include <vector>
#include <json/value.h>
#include "TreeMeshes.h"
#include "../math/AABB.h"
#include "declarative/SensorModelParameters.h"
#include "../planning/RobotModel.h"
#include "tree_models.h"
#include "LoadedTreeModel.h"
#include "declarative/PointScanExperiment.h"

namespace mgodpl::declarative {

	/**
	 * An instantiation of PointScanEvalParameters, providing a full environment for a point scanning experiment,
	 * including the robot model, the tree model, the fruit models, and the scannable points.
	 *
	 * This is read-only: state-related information should be instantiated in a separate context.
	 */
	struct PointScanEnvironment {
		/// The robot model to use.
		const robot_model::RobotModel robot;
		/// The tree model to use, including the meshes and pre-computed properties.
		const std::shared_ptr<const experiments::LoadedTreeModel> tree_model;
		/// The mesh of the leaves, possibly re-scaled to simulate different canopy densities.
		const Mesh scaled_leaves;
		/// The centers of all the fruits.
		const FruitModels fruit_models;
		/// The scannable points per fruit, including surface normal. One vector per fruit mesh, corresponding to tree_model.
		const std::vector<std::vector<SurfacePoint>> scannable_points;
		/// The occlusion model to use to accelerate occlusion checks.
		const std::shared_ptr<const MeshOcclusionModel> mesh_occlusion_model;
		/// The initial state of the robot.
		const RobotState initial_state;
	};

	std::vector<std::vector<SurfacePoint>> generate_scannable_points(const FruitModels &fruit_models,
																	 size_t n_points_per_fruit,
																	 random_numbers::RandomNumberGenerator &rng);

	/**
	 * Obtain an environment instance for a given set of parameters.
	 *
	 * Parts of it will be retrieved from the cache if it already exists, or created and stored in the cache if it does not.
	 * This is potentially a relatively expensive operation, as it may involve loading meshes from disk and pre-computing properties.
	 * A mutex protects the cache from concurrent access, so this function is thread-safe.
	 *
	 * Caching applies to only the deterministic parts of this operation, not to point sampling.
	 *
	 * @param params 		The parameters to use to create the environment.
	 * @param rng 			A random number generator to use for sampling points.
	 * @return 				A PointScanEnvironment instance for the given parameters.
	 */
	PointScanEnvironment create_environment(const PointScanEvalParameters &params, experiments::TreeModelCache &tree_model_cache);
}

#endif //MGODPL_DECLARATIVE_ENVIRONMENT_H
