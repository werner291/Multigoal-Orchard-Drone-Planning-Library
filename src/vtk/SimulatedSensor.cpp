//
// Created by werner on 5-10-22.
//

#include "SimulatedSensor.h"
#include "../utilities/vtk.h"
#include "../exploration/VtkToPointCloud.h"
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkRendererSource.h>

SimulatedSensor::SimulatedSensor(bool showWindow) {

	sensorRenderer = buildSensorRenderer();
	sensorWindow = buildSensorRenderWindow(sensorRenderer);

	if (!showWindow) {
		sensorWindow->OffScreenRenderingOn();
	}

	rendererSource->DepthValuesOn();
	rendererSource->SetInput(sensorRenderer);

	depthToPointCloud->SetInputConnection(0, rendererSource->GetOutputPort());
	depthToPointCloud->SetCamera(sensorRenderer->GetActiveCamera());

}

void SimulatedSensor::addActor(vtkActor *actor) {
	sensorRenderer->AddActor(actor);
}

void SimulatedSensor::addActorCollection(vtkActorCollection *actors) {
	addActorCollectionToRenderer(actors, sensorRenderer);
}

vtkAlgorithmOutput *SimulatedSensor::getPointCloudOutputPort() const {
	return depthToPointCloud->GetOutputPort();
}

vtkPolyData *SimulatedSensor::getPointCloud() const {
	return depthToPointCloud->GetOutput();
}

SegmentedPointCloud::ByType SimulatedSensor::renderSnapshot(const Eigen::Isometry3d &from_pose) {

	setCameraFromEigen(from_pose, sensorRenderer->GetActiveCamera());

	sensorWindow->Render();

	rendererSource->Update();
	depthToPointCloud->Update();

	return segmentPointCloudData(getPointCloud()).split_by_type();

}
