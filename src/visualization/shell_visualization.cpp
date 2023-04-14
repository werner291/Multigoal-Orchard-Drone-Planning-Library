// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <vtkProperty.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>

#include "shell_visualization.h"

vtkSmartPointer<vtkActor> mkSphereShellActor(const WorkspaceSphereShell &sphereshell) {
	vtkNew<vtkSphereSource> sphereSource;
	sphereSource->SetCenter(sphereshell.getCenter().x(), sphereshell.getCenter().y(), sphereshell.getCenter().z());
	sphereSource->SetRadius(sphereshell.getRadius());
	sphereSource->SetThetaResolution(32);
	sphereSource->SetPhiResolution(16);

	vtkNew<vtkPolyDataMapper> sphereMapper;
	sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

	vtkNew<vtkActor> sphereActor;
	sphereActor->SetMapper(sphereMapper);

	sphereActor->GetProperty()->SetColor(0.8, 0.8, 0.8);
	sphereActor->GetProperty()->SetOpacity(0.5);

	return sphereActor;
}
