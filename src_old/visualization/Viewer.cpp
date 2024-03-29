
#include "Viewer.h"
#include "../utilities/vtk.h"

#include <vtkLightCollection.h>
#include <vtkLight.h>
#include <vtkProperty.h>
#include <vtkInteractorStyle3D.h>
#include <vtkCamera.h>

Viewer::Viewer(const SimplifiedOrchard &orchard, VtkRobotmodel &robotModel, const std::vector<vtkActor *> &actors)
		: targetToHullLineSegments(1, 0, 0),
		  visitOrderVisualization(1.0, 0.5, 0.5),
		  ee_trace_visualization(0.0, 0.0, 1.0),
		  obstacleSurfacepoints(0.5, 0.25, 0.125) {

	viewerRenderer->SetBackground(0.1, 0.1, 0.5);
	viewerRenderer->ResetCamera();

	viewerRenderer->RemoveAllLights();

	vtkLight *naturalLight = viewerRenderer->MakeLight();
	naturalLight->SetDiffuseColor(1.0, 1.0, 1.0);
	naturalLight->SetDirectionAngle(70, 45);
	viewerRenderer->AddLight(naturalLight);

	visualizerWindow->SetSize(800,600);
	visualizerWindow->SetWindowName("PointCloud");
	visualizerWindow->AddRenderer(viewerRenderer);

	renderWindowInteractor->SetRenderWindow(visualizerWindow);
	renderWindowInteractor->CreateRepeatingTimer(33);

	addActor(convexHullActor.actor);
//	addActor(targetToHullLineSegments.getActor());
//	addActor(visitOrderVisualization.getActor());
	addActor(ee_trace_visualization.getActor());

	addActorCollection(buildOrchardActors(orchard, false));
	addActorCollection(robotModel.getLinkActors());

	for (auto &actor: actors) {
		addActor(actor);
	}

	viewerRenderer->GetActiveCamera()->SetPosition(8, 4, 2.5);
	viewerRenderer->GetActiveCamera()->SetFocalPoint(2, 0, 1.5);
	viewerRenderer->GetActiveCamera()->SetViewUp(0, 0, 1);

	obstacleSurfacepoints.getActor()->SetPosition(0.0, 5.0, 0.0);

	addActor(obstacleSurfacepoints.getActor());
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
	viewerRenderer->GetActiveCamera()->SetViewUp(0, 0, 1);
	visualizerWindow->Render();
}

void Viewer::setIntervalCallback(const std::function<void()> &callback) {
	vtkNew<vtkFunctionalCallback> cb;
	cb->setEventId(vtkCommand::TimerEvent);
	cb->setCallback(callback);
	renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, cb);
}

