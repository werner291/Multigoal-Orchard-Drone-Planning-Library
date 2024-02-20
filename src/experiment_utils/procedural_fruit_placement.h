// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 2/20/24.
//

#ifndef MGODPL_PROCEDURAL_FRUIT_PLACEMENT_H
#define MGODPL_PROCEDURAL_FRUIT_PLACEMENT_H

#include <vector>
#include <random_numbers/random_numbers.h>
#include "../math/Vec3.h"
#include "TreeMeshes.h"

namespace mgodpl {

	const double FRUIT_RADIUS = 0.04;

	/**
	 * Generate fruit center points uniformly distributed on the trunk of the tree.
	 *
	 * The distribution samples points uniformly by area on the trunk of the tree (so thinner twigs are less likely to have fruit).
	 *
	 * @param tree_model 		The tree model
	 * @param num_fruits 		The number of fruits to generate
	 * @param rng 				A random number generator
	 * @return 					A vector of fruit center points
	 */
	std::vector<math::Vec3d> generate_fruit_locations(const mgodpl::tree_meshes::TreeMeshes &tree_model,
													  size_t num_fruits,
													  random_numbers::RandomNumberGenerator &rng);

	/**
	 * Generate fruit growing in "bunches" on the tree.
	 *
	 * @param tree_model 			The tree model
	 * @param num_clusters			The number of clusters of fruit to generate
	 * @param min_cluster_size		The minimum number of fruit in a cluster
	 * @param max_cluster_size		The maximum number of fruit in a cluster
	 * @param rng 					A random number generator
	 * @return 						A vector of fruit center points
	 */
	std::vector<math::Vec3d> generate_fruit_clusters(const mgodpl::tree_meshes::TreeMeshes &tree_model,
													 size_t num_clusters,
													 size_t min_cluster_size,
													 size_t max_cluster_size,
													 bool bias_towards_small_clusters,
													 random_numbers::RandomNumberGenerator &rng);
}

#endif //MGODPL_PROCEDURAL_FRUIT_PLACEMENT_H
