
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkNew.h>
#include <vtkProperty.h>
#include <vtkWindowToImageFilter.h>
#include <vtkDepthImageToPointCloud.h>
#include <vtkPointData.h>
#include <vtkCallbackCommand.h>

#include <utility>

#include <range/v3/view/enumerate.hpp>

#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include <geometric_shapes/shape_operations.h>
#include <vtkSphereSource.h>

#include "../utilities/experiment_utils.h"
#include "../vtk/VtkRobotModel.h"
#include "../utilities/load_mesh.h"
#include "../utilities/moveit.h"
#include "../exploration/StupidAlgorithm.h"
#include "../exploration/ColorEncoding.h"
#include "../exploration/VtkToPointCloud.h"
#include "../utilities/vtk.h"
#include "../vtk/SimulatedSensor.h"
#include "../vtk/Viewer.h"
#include "../utilities/msgs_utilities.h"
#include "../exploration/DynamicBoundingSphereAlgorithm.h"
#include "../StreamingConvexHull.h"

moveit::core::RobotState mkInitialState(const moveit::core::RobotModelPtr &drone);

std::vector<ScanTargetPoint> buildScanTargetPoints(const shape_msgs::msg::Mesh &mesh, size_t n);

SegmentedPointCloud generateInitialCloud(const SimplifiedOrchard &orchard);

int main(int, char*[]) {

	auto drone = loadRobotModel();
	moveit::core::RobotState current_state = mkInitialState(drone);

	VtkRobotmodel robotModel(drone);
	robotModel.applyState(current_state);

	SimplifiedOrchard orchard = {
		{
			{ { 0.0, 0.0 }, loadTreeMeshes("appletree") },
		}
	};

	auto collision_world = std::make_shared<collision_detection::World>();

	for (const auto& [tree_id, tree] : orchard.trees | ranges::views::enumerate) {

		Eigen::Isometry3d tree_pose = Eigen::Isometry3d::Identity();
		tree_pose.translation() = Eigen::Vector3d(tree.first.x(), tree.first.y(), 0);
		collision_world->addToObject("tree_" + std::to_string(tree_id) + "_trunk",
									 shapes::ShapeConstPtr(shapes::constructShapeFromMsg(tree.second.trunk_mesh)),
									 tree_pose);
	}

	collision_detection::CollisionEnvFCL collision_env(drone, collision_world);

	auto surface_points = buildScanTargetPoints(orchard.trees[0].second.fruit_mesh, 2000);
	
	ScannablePointsIndex scannablePointsIndex(surface_points);

	vtkNew<vtkPolyData> fruitSurfacePolyData = mkVtkPolyDataFromScannablePoints(surface_points);

	vtkNew<vtkActor> fruitSurfacePointsActor = constructSimplePolyDataPointCloudActor(fruitSurfacePolyData);

	auto orchard_actors = buildOrchardActors(orchard);

	SimulatedSensor sensor;
	sensor.addActorCollection(orchard_actors);
	sensor.addActorCollection(robotModel.getLinkActors());

	vtkNew<vtkActor> pointCloudActor = buildDepthImagePointCloudActor(sensor.getPointCloudOutputPort());

	StreamingConvexHull convexHull = StreamingConvexHull::fromSpherifiedCube(3);

	vtkNew<vtkPolyDataMapper> mapper;
	mapper->SetInputData(rosMeshToVtkPolyData(convexHull.toMesh()));

	vtkNew<vtkActor> actor;
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(1.0, 0.0, 1.0);

	Viewer viewer;
	viewer.addActorCollection(orchard_actors);
	viewer.addActorCollection(robotModel.getLinkActors());
	viewer.addActor(fruitSurfacePointsActor);
	viewer.addActor(actor);
	// For an unknown
	viewer.addActor(pointCloudActor);

	double pathProgressT = 0.0;
	std::optional<robot_trajectory::RobotTrajectory> currentTrajectory;

	DynamicBoundingSphereAlgorithm dbsa([&](robot_trajectory::RobotTrajectory trajectory) {
		currentTrajectory = std::move(trajectory);
		pathProgressT = 0.0;
	});

	dbsa.updatePointCloud(current_state, generateInitialCloud(orchard));

	auto callback = [&]() {

		if (currentTrajectory) {

			pathProgressT += 0.005;

			setStateToTrajectoryPoint(current_state, pathProgressT, *currentTrajectory);

			collision_detection::CollisionRequest collision_request;
			collision_detection::CollisionResult collision_result;
			collision_env.checkRobotCollision(collision_request, collision_result, current_state);

			if (collision_result.collision) {
				std::cout << "Oh no!" << std::endl;
			}

		}

		robotModel.applyState(current_state);

		Eigen::Isometry3d eePose = current_state.getGlobalLinkTransform("end_effector");
		sensor.requestRender(eePose);

		// Nullptr check to avoid a race condition where the callback is called before the first point cloud is rendered.
		// FIXME: Ideally, I'd like to make sure the algorithm sees the point cloud exactly once.
		if (sensor.getPointCloud()->GetPoints() != nullptr) {

			SegmentedPointCloud segmentedPointCloud = segmentPointCloudData(sensor.getPointCloud());

			dbsa.updatePointCloud(current_state, segmentedPointCloud);

			std::vector<Eigen::Vector3d> points;

			for (const auto &item: segmentedPointCloud.points) {
				if (item.type == SegmentedPointCloud::PT_SOFT_OBSTACLE && item.position.z() > 1.0e-6) {
					points.push_back(item.position);
				}
			}

			if (convexHull.addPoints(points))
			{
				auto mesh = convexHull.toMesh();
				mapper->SetInputData(rosMeshToVtkPolyData(convexHull.toMesh()));
				mapper->Modified();
			}

			auto scanned_points = scannablePointsIndex
					.findScannedPoints(eePose.translation(),eePose.rotation() *Eigen::Vector3d(0, 1, 0),M_PI / 4.0,0.1,segmentedPointCloud);

			for (auto &point: scanned_points) {
				fruitSurfacePolyData->GetPointData()->GetScalars()->SetTuple3((vtkIdType) point, 255, 0, 255);
				fruitSurfacePolyData->Modified();
			}
		}

		viewer.requestRender();

	};

	viewer.setIntervalCallback(callback);

	viewer.start();

	return EXIT_SUCCESS;
}

SegmentedPointCloud generateInitialCloud(const SimplifiedOrchard &orchard) {// Pick a random fruit mesh vertex.
	ompl::RNG rng;
	auto fruitMeshVertex = orchard.trees[0].second.fruit_mesh.vertices[rng.uniformInt(0, orchard.trees[0].second.fruit_mesh.vertices.size() - 1)];

	SegmentedPointCloud initialCloud {
			{SegmentedPointCloud::Point{SegmentedPointCloud::PT_TARGET,
										{fruitMeshVertex.x, fruitMeshVertex.y, fruitMeshVertex.z},

			}}};
	return initialCloud;
}


moveit::core::RobotState mkInitialState(const moveit::core::RobotModelPtr &drone) {
	moveit::core::RobotState current_state(drone);
	current_state.setVariablePositions({
											   5.0, 0.0, 1.5,
											   0.0, 0.0, 0.0, 1.0,
											   0.0, 0.0, 0.0, 0.0
									   });
	current_state.update();
	return current_state;
}



