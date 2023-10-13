
#pragma once

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vector>
#include <string>
#include <utility>

using LabeledActors = std::vector<std::pair<std::vector<vtkSmartPointer<vtkActor>>, std::string>>;

struct TreeMeshes;

LabeledActors treeMeshesToLabeledActors(const TreeMeshes &treeMeshes);