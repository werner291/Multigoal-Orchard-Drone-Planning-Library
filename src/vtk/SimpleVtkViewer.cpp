// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "SimpleVtkViewer.h"
#include "../utilities/vtk.h"

SimpleVtkViewer::SimpleVtkViewer() {
	// Set up the render window.
	visualizerWindow->SetSize(800, 600);
	visualizerWindow->SetWindowName("PointCloud");
	visualizerWindow->AddRenderer(viewerRenderer);

	// Set up the render window interactor.
	renderWindowInteractor->SetRenderWindow(visualizerWindow);
	renderWindowInteractor->CreateRepeatingTimer(33);

	addTimerCallback([&]() {
		renderWindowInteractor->GetRenderWindow()->Render();
	});
}

void SimpleVtkViewer::addActor(vtkActor *actor) {
	viewerRenderer->AddActor(actor);
}

void SimpleVtkViewer::addTimerCallback(std::function<void()> callback) {
	// Set the "main loop" callback to be called every frame.
	vtkNew<vtkFunctionalCallback> cb;
	cb->setEventId(vtkCommand::TimerEvent);
	cb->setCallback(callback);
	renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, cb);
}

void SimpleVtkViewer::start() {
	renderWindowInteractor->Start();
}