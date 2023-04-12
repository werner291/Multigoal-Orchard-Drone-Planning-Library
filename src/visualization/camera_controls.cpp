// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 12-4-23.
//

#include <vtkCallbackCommand.h>
#include <vtkCamera.h>

#include "camera_controls.h"

void enforceCameraUp(vtkRenderer *renderer, vtkRenderWindowInteractor *interactor) {

	// Create a vtkCallbackCommand for the observer
	vtkNew<vtkCallbackCommand> resetViewUpCallback;

	// Set the callback function using a lambda
	resetViewUpCallback->SetCallback([](vtkObject *caller, unsigned long eventId, void *clientData, void *callData) {
		auto *renderer = static_cast<vtkRenderer *>(clientData);
		vtkCamera *camera = renderer->GetActiveCamera();
		camera->SetViewUp(0.0, 0.0, 1.0);
	});

	renderer->GetActiveCamera()->SetPosition(10.0, 0.0, 3.0);
	renderer->GetActiveCamera()->SetFocalPoint(0.0, 0.0, 2.0);
	renderer->GetActiveCamera()->SetViewUp(0.0, 0.0, 1.0);

	// Set the clientData (the renderer in this case)
	resetViewUpCallback->SetClientData(renderer);

	// Add an observer for the EndInteractionEvent
	unsigned long observerId = interactor->AddObserver(vtkCommand::InteractionEvent, resetViewUpCallback);
}