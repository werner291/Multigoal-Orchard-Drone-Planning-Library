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
#include "../TreeMeshes.h"
#include "../../math/AABB.h"
#include "DeclarativeExperimentParameters.h"
#include "../../planning/RobotModel.h"

namespace mgodpl::declarative {

	/**
	 * The result of a tree model loaded from disk, including the geometry and a number of pre-computed properties.
	 */
	struct LoadedTreeModel {

		/// The tree meshes, including the leaves, fruits, and branches. (TODO: need to have fruit positions be part of this?)
		mgodpl::tree_meshes::TreeMeshes meshes;

		/// The root points of the leaves, for easy re-scaling.
		std::vector<math::Vec3d> root_points;

		/// The axis-aligned bounding box of the leaves.
		math::AABBd leaves_aabb;

		/// A rough measure of the radius of the tree's canopy.
		double canopy_radius;

		/**
		 * Load a tree model from disk by name, and pre-compute some properties.
		 *
		 * See `mgodpl::tree_meshes::loadTreeMeshes` for more information about the naming and loading.
		 *
		 * @param name 	The name of the tree model to load.
		 * @return 		A LoadedTreeModel object representing the tree model.
		 */
		static LoadedTreeModel from_name(const std::string &name);

	};

	/**
	 * The environmental parameters for a given point scanning experiment, assuming a static environment.
	 *
	 * This serves as a POD specification for the environment and objective of the experiment,
	 * and us meant to be stored alongside the results in order to reproduce them and understand
	 * the impact of environmental parameters on the results.
	 */
	struct PointScanEvalParameters {

		/// Parameters about the tree model, including which model to use and the number of fruits to scan.
		TreeModelParameters tree_params;

		/// Parameters about the sensor, including the field of view and the maximum view distance.
		SensorScalarParameters sensor_params;

		/// How many points to use per fruit. (TODO: this is more an implementation detail, does it belong here?)
		size_t n_scannable_points_per_fruit = 200;

	};

	Json::Value toJson(const PointScanEvalParameters &params);

	/**
	 * An instantiation of PointScanEvalParameters.
	 *
	 * Note: all struct members are references to allow re-use of the same objects.
	 *
	 * This should be read-only and therefore safe to use in a multi-threaded context.
	 */
	struct PointScanEnvironment {
		/// The robot model to use.
		const robot_model::RobotModel &robot;
		/// The tree model to use, including the meshes and pre-computed properties.
		const std::shared_ptr<const LoadedTreeModel> tree_model;
		/// The mesh of the leaves, possibly re-scaled to simulate different canopy densities.
		const shape_msgs::msg::Mesh scaled_leaves;
		/// The scannable points per fruit, including surface normal. One vector per fruit mesh, corresponding to tree_model.
		const std::vector<std::vector<SurfacePoint>> scannable_points;
		/// The occlusion model to use to accelerate occlusion checks.
		const std::shared_ptr<const MeshOcclusionModel> mesh_occlusion_model;
	};

	std::vector<shape_msgs::msg::Mesh> random_subset_of_meshes(random_numbers::RandomNumberGenerator &rng,
															   const std::shared_ptr<const LoadedTreeModel> &tree_model,
															   const RandomSubset &subset_params);

	/**
	 * This struct serves as a cache to create simulation environments for a given set of parameters.
	 */
	struct EnvironmentInstanceCache {

		/// A mutex to protect the cache from concurrent access.
		std::mutex mutex{};

		/// The robot model to use.
		const robot_model::RobotModel &robot;

		/// A cache map of tree model names to their loaded instances.
		std::unordered_map<std::string, std::shared_ptr<LoadedTreeModel>> tree_models = {};

		/**
		 * Obtain a tree model and precompute some properties; this operation is cached.
		 *
		 * Access to the cache is thread-safe via a mutex.
		 *
		 * @param tree_name		The name of the tree model to load.
		 * @return 				A shared pointer to the loaded tree model.
		 */
		std::shared_ptr<const LoadedTreeModel> obtain_tree_model(const std::string &tree_name);

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
		PointScanEnvironment create_environment(const PointScanEvalParameters &params);
	};
}

#endif //MGODPL_DECLARATIVE_ENVIRONMENT_H
