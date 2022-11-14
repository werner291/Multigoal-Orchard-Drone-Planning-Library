

#include "VisualizationSpecifics.h"
#include "../utilities/vtk.h"

#include <vtkProperty.h>
#include <vtkPointData.h>

FruitSurfaceScanTargetsActor::FruitSurfaceScanTargetsActor(const std::vector<ScanTargetPoint> &pt) : fruitSurfacePolyData(mkVtkPolyDataFromScannablePoints(pt)) {
	vtkNew<vtkPolyDataMapper> fruitSurfacePointsMapper;
	fruitSurfacePointsMapper->SetInputData(fruitSurfacePolyData);
	fruitSurfacePointsActor->SetMapper(fruitSurfacePointsMapper);
	fruitSurfacePointsActor->GetProperty()->SetPointSize(5);
}

void FruitSurfaceScanTargetsActor::markAsScanned(const std::vector<size_t> &indices) { // NOLINT(readability-make-member-function-const)
	for (auto &point: indices) {
		fruitSurfacePolyData->GetPointData()->GetScalars()->SetTuple3((vtkIdType) point, 255, 0, 255);
		fruitSurfacePolyData->Modified();
	}
}

ConvexHullActor::ConvexHullActor() {
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(0.0, 1.0, 0.0);
	actor->GetProperty()->SetOpacity(0.8);
}

void ConvexHullActor::update(const shape_msgs::msg::Mesh &mesh) {
	mapper->SetInputData(rosMeshToVtkPolyData(mesh));
}

Viewer buildViewer(const SimplifiedOrchard &orchard,
				   VtkRobotmodel &robotModel,
				   const std::vector<vtkActor*>& actors) {

	Viewer viewer;
	viewer.addActorCollection(buildOrchardActors(orchard, false));
	viewer.addActorCollection(robotModel.getLinkActors());

	for (auto &actor: actors) {
		viewer.addActor(actor);
	}

	viewer.viewerRenderer->GetActiveCamera()->SetPosition(4,4,2.0);
	viewer.viewerRenderer->GetActiveCamera()->SetFocalPoint(0,0,1.5);
	viewer.viewerRenderer->GetActiveCamera()->SetViewUp(0,0,1);

	return viewer;
}

SimulatedSensor buildSensorSimulator(const SimplifiedOrchard &orchard, VtkRobotmodel &robotModel) {
	SimulatedSensor sensor;
	sensor.addActorCollection(buildOrchardActors(orchard, true));
	sensor.addActorCollection(robotModel.getLinkActors());
	return sensor;
}
