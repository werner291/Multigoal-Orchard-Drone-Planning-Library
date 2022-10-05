//
// Created by werner on 5-10-22.
//

#include "SimulatedSensor.h"
#include "../utilities/vtk.h"


SimulatedSensor::SimulatedSensor() {

	sensorRenderer = buildSensorRenderer();
	sensorWindow = buildSensorRenderWindow(sensorRenderer);
	depthToPointCloud = extractPointCloudFromRenderer(sensorRenderer);

	sensorWindow->Render();
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

void SimulatedSensor::requestRender(const Eigen::Isometry3d &from_pose) {
	setCameraFromEigen(from_pose, sensorRenderer->GetActiveCamera());
	sensorWindow->Render();
}


