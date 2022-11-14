
#include "Viewer.h"
#include "../utilities/vtk.h"

#include <vtkLightCollection.h>
#include <vtkLight.h>
#include <vtkProperty.h>

Viewer::Viewer() {
	viewerRenderer->SetBackground(0.1, 0.1, 0.5);
	viewerRenderer->ResetCamera();
//
	viewerRenderer->RemoveAllLights();

	vtkLight* naturalLight = viewerRenderer->MakeLight();
	naturalLight->SetDiffuseColor(1.0, 1.0, 1.0);
	naturalLight->SetDirectionAngle(70, 45);
	viewerRenderer->AddLight(naturalLight);

//	vtkLight::SafeDownCast(viewerRenderer->GetLights()->GetItemAsObject(0))->SetAmbientColor(0.0, 0.0, 1.0);

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
