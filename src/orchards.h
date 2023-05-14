// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 14-5-23.
//

#ifndef NEW_PLANNERS_ORCHARDS_H
#define NEW_PLANNERS_ORCHARDS_H

#include <Eigen/Core>
#include "TreeMeshes.h"

static const double SPACING = 1.0;

struct TreeAtPosition {
	TreeMeshes meshes; // The meshes of the tree.
	Eigen::Vector3d position; // The position of the tree.
	double rotation_angle; // Rotation angle about the z-axis in radians.
};

/**
 * @brief Creates a row of trees in an orchard with random rotations about the z-axis.
 *
 * This function samples trees from a given vector of TreeMeshes with replacement,
 * and places them at a fixed interval along the X-axis.
 *
 * @param treeModels A constant reference to a vector of TreeMeshes.
 * @param interval The fixed interval between trees along the X-axis.
 * @param num_samples The number of trees to sample and place in the row.
 * @return A vector of TreeAtPosition objects representing a row of trees.
 */
std::vector<TreeAtPosition> createOrchardRow(const std::vector<TreeMeshes>& treeModels, double interval, int num_samples);

/**
 * @brief Merge the meshes of multiple trees into a single TreeMeshes object.
 *
 * @param trees The trees to merge.
 * @return The merged TreeMeshes object.
 */
TreeMeshes mergeTreeMeshes(const std::vector<TreeAtPosition>& trees);

#endif //NEW_PLANNERS_ORCHARDS_H
