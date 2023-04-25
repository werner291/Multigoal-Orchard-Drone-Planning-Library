// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "SimpleVtkViewer.h"
#include "../utilities/vtk.h"
#include <vtkProperty.h>

#include "camera_controls.h"

SimpleVtkViewer::SimpleVtkViewer() {
	// Set up the render window.
	visualizerWindow->SetSize(800, 600);
	visualizerWindow->SetWindowName("Robot Path Planning");
	visualizerWindow->AddRenderer(viewerRenderer);

	// Set up the render window interactor.
	renderWindowInteractor->SetRenderWindow(visualizerWindow);
	renderWindowInteractor->CreateRepeatingTimer(33);

	viewerRenderer->SetBackground(0.5, 0.8, 0.9);

	addTimerCallback([&]() {
		renderWindowInteractor->GetRenderWindow()->Render();

		if (videoRecorder.has_value()) {
			videoRecorder->exportFrame();
		}
	});

	enforceCameraUp(viewerRenderer, renderWindowInteractor);

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

void SimpleVtkViewer::startRecording(const std::string &filename) {
	videoRecorder.emplace(visualizerWindow, filename);
}

void SimpleVtkViewer::stop() {

	renderWindowInteractor->TerminateApp();

	if (videoRecorder.has_value()) {
		videoRecorder->finish();
	}
}

void SimpleVtkViewer::discardVideo() {
	videoRecorder.reset();
}

void SimpleVtkViewer::addMesh(const shape_msgs::msg::Mesh &mesh, const Eigen::Vector3d &color, double opacity) {

	auto actor = createActorFromMesh(mesh);

	actor->GetProperty()->SetColor(color[0], color[1], color[2]);

	if (opacity < 1.0) {
		actor->GetProperty()->SetOpacity(opacity);
	}

	addActor(actor);

}
