// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "SimpleVtkViewer.h"
#include "../utilities/vtk.h"

SimpleVtkViewer::SimpleVtkViewer() {
	// Set up the render window.
	visualizerWindow->SetSize(800, 600);
	visualizerWindow->SetWindowName("Robot Path Planning");
	visualizerWindow->AddRenderer(viewerRenderer);

	// Set up the render window interactor.
	renderWindowInteractor->SetRenderWindow(visualizerWindow);
	renderWindowInteractor->CreateRepeatingTimer(33);

	addTimerCallback([&]() {
		renderWindowInteractor->GetRenderWindow()->Render();
		viewerRenderer->GetActiveCamera()->SetViewUp(0, 0, 1);
		viewerRenderer->GetActiveCamera()->SetFocalPoint(0, 0, 1.5);
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

void SimpleVtkViewer::addActorCollection(vtkActorCollection *actors) {
	for (int i = 0; i < actors->GetNumberOfItems(); i++) {
		viewerRenderer->AddActor(vtkActor::SafeDownCast(actors->GetItemAsObject(i)));
	}
}
