
#include "Viewer.h"
#include "../utilities/vtk.h"

Viewer::Viewer() {
	viewerRenderer->SetBackground(0.1, 0.1, 0.5);
	viewerRenderer->ResetCamera();

	vtkNew<vtkLight> naturalLight;
	naturalLight->SetAmbientColor(0.0, 0.0, 0.0);
	viewerRenderer->ClearLights();
	viewerRenderer->AddLight(naturalLight);

	visualizerWindow->SetSize(800,600);
	visualizerWindow->SetWindowName("PointCloud");
	visualizerWindow->AddRenderer(viewerRenderer);

	renderWindowInteractor->SetRenderWindow(visualizerWindow);
	renderWindowInteractor->CreateRepeatingTimer(33);
}

void Viewer::addActor(vtkActor *actor) {
	viewerRenderer->AddActor(actor);
}

void Viewer::addActorCollection(vtkActorCollection *actors) {
	addActorCollectionToRenderer(actors, viewerRenderer);
}

void Viewer::start() {
	renderWindowInteractor->Start();
}

void Viewer::requestRender() {
	visualizerWindow->Render();
}

void Viewer::setIntervalCallback(const std::function<void()>& callback) {
	vtkNew<vtkFunctionalCallback> cb;
	cb->setEventId(vtkCommand::TimerEvent);
	cb->setCallback(callback);
	renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, cb);
}
