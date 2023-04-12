// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 12-4-23.
//

#ifndef NEW_PLANNERS_SHELL_VISUALIZATION_H
#define NEW_PLANNERS_SHELL_VISUALIZATION_H

#include <vtkActor.h>
#include <vtkNew.h>

#include "../shell_space/SphereShell.h"

vtkNew<vtkActor> mkSphereShellActor(const WorkspaceSphereShell &sphereshell);

#endif //NEW_PLANNERS_SHELL_VISUALIZATION_H
