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
	 * @brief finds the closest point on the trunk for every leaf of the tree and assigns a Vec3d with the coordinates of that root point to every other vertex of the leaf.
	 */
	std::vector<math::Vec3d> leaf_root_points(const mgodpl::tree_meshes::TreeMeshes &tree_meshes);

	/**
	 * @brief Scales the leaves of a tree around their root points.
	 *
	 * This function takes as input the tree meshes, the root points for each leaf, and a scale factor. It creates a copy of the leaves mesh and then iterates over all the vertices in the mesh.
	 * For each vertex, it retrieves the corresponding root point and scales the vertex around this root point by the given scale factor. The scaled vertex is then written back to the mesh.
	 * The function finally returns the modified leaves mesh.
	 *
	 * @param tree_meshes The tree meshes which include the trunk mesh and the leaves mesh.
	 * @param leaf_root_vertex A vector of math::Vec3d objects representing the root point for each leaf.
	 * @param scale_factor The factor by which to scale the leaves.
	 * @return A shape_msgs::msg::Mesh object representing the leaves mesh after scaling.
	 */
	shape_msgs::msg::Mesh scale_leaves(
			const mgodpl::tree_meshes::TreeMeshes &tree_meshes,
			const std::vector<math::Vec3d> &leaf_root_vertex,
			double scale_factor
			);

}

#endif //MGODPL_LEAF_SCALING_H
