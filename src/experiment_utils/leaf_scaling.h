// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 26-1-24.
//

#ifndef MGODPL_LEAF_SCALING_H
#define MGODPL_LEAF_SCALING_H

#include <vector>
#include <cstddef>
#include "TreeMeshes.h"
#include "mesh_connected_components.h"

namespace mgodpl {

	/**
	 * @brief Finds the closest point on the trunk for every leaf of the tree and assigns the index of that vertex to every other vertex of the leaf.
	 *
	 * This function first builds an AABB tree from the trunk mesh. Then, for each connected component in the leaves mesh,
	 * it finds the leaf closest to the trunk and assigns the index of the closest leaf to all vertices in the component.
	 *
	 * @param tree_meshes The tree meshes for one tree.
	 * @return A vector of indices, one for every vertex of the leaves mesh.
	 */
	std::vector<size_t> leaf_root_vertex(const mgodpl::tree_meshes::TreeMeshes &tree_meshes);

	/**
	 * @brief Scales the leaves of the tree based on the distance to the trunk, using the result of leaf_root_vertex.
	 *
	 * @param tree_meshes The tree meshes for one tree.
	 * @param leaf_root_vertex The vector of indices, one for every vertex of the leaves mesh.
	 * @param scale_factor The factor to scale the leaves with.
	 */
	shape_msgs::msg::Mesh scale_leaves(const mgodpl::tree_meshes::TreeMeshes &tree_meshes, const std::vector<size_t> &leaf_root_vertex, double scale_factor);

}

#endif //MGODPL_LEAF_SCALING_H
