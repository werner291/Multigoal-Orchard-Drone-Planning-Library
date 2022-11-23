

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



SimulatedSensor buildSensorSimulator(const SimplifiedOrchard &orchard, VtkRobotmodel &robotModel) {
	SimulatedSensor sensor;
	sensor.addActorCollection(buildOrchardActors(orchard, true));
	sensor.addActorCollection(robotModel.getLinkActors());
	return sensor;
}

std::vector<Eigen::Vector3d> extractEndEffectorTrace(const robot_trajectory::RobotTrajectory &trajectory) {
	std::vector<Eigen::Vector3d> points;

	for (size_t wp_idx = 0; wp_idx < trajectory.getWayPointCount(); wp_idx++) {
		points.emplace_back(trajectory.getWayPoint(wp_idx).getGlobalLinkTransform("end_effector").translation());
	}
	return points;
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
extractTargetHullPointsSegments(const DynamicMeshHullAlgorithm &dbsa) {
	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> lines;

	for (const auto &[original, projected,_1,_2]: dbsa.getTargetPointsOnChullSurface()) {
		lines.push_back({original.position, projected});
	}
	return lines;
}

std::vector<Eigen::Vector3d> extractVisitOrderPoints(const DynamicMeshHullAlgorithm &dbsa, Eigen::Isometry3d &eePose) {
	std::vector<Eigen::Vector3d> visit_order_points {
			eePose.translation()
	};
	for (const auto &point: dbsa.getVisitOrdering().getVisitOrdering()) {
		visit_order_points.push_back(dbsa.getTargetPointsOnChullSurface()[point].hull_location);
	}
	return visit_order_points;
}
