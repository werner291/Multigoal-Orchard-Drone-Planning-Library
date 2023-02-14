// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.


#ifndef NEW_PLANNERS_VISUALIZE_PATH_H
#define NEW_PLANNERS_VISUALIZE_PATH_H

#include "../RobotPath.h"
#include "../TreeMeshes.h"

/**
 * Quickly visualizes a path in VTK, toghether with the tree meshes.
 *
 * @param meshes A reference to a TreeMeshes object containing tree meshes to be visualized.
 * @param path A reference to a RobotPath object containing a path to be visualized.
 */
void quickVisualizePath(const TreeMeshes &meshes, const RobotPath &path);

#endif //NEW_PLANNERS_VISUALIZE_PATH_H
