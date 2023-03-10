// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10-3-23.
//

#ifndef NEW_PLANNERS_ORCHARD_ACTORS_H
#define NEW_PLANNERS_ORCHARD_ACTORS_H

#include <vector>
#include <vtkActorCollection.h>
#include "../TreeMeshes.h"
#include "SimpleVtkViewer.h"
#include "../utilities/discoverability_specifications.h"

std::vector<vtkActor *> createActors(const TreeMeshes &meshes, SimpleVtkViewer &viewer);

#endif //NEW_PLANNERS_ORCHARD_ACTORS_H
