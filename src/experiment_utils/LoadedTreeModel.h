// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/12/24.
//

#ifndef MGODPL_LOADEDTREEMODEL_H
#define MGODPL_LOADEDTREEMODEL_H

#include <unordered_map>
#include <mutex>
#include "TreeMeshes.h"
#include "../math/AABB.h"

namespace mgodpl::experiments {

	/**
	 * The result of a tree model loaded from disk, including the geometry and a number of pre-computed properties.
	 *
	 * It relates to the `mgodpl::tree_meshes::TreeMeshes` structure, but adds a bunch of properties
	 * that are useful for the point scanning experiments.
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
	 * A cache of tree models; contains a mutex to protect the cache from concurrent access.
	 */
	class TreeModelCache {
		std::mutex mutex;
		std::unordered_map<std::string, std::shared_ptr<const LoadedTreeModel>> cache;

	public:
		std::shared_ptr<const LoadedTreeModel> obtain_by_name(const std::string &name);
	};

} // mgodpl::experiments

#endif //MGODPL_LOADEDTREEMODEL_H
