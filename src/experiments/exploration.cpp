
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkWindowToImageFilter.h>
#include <vtkDepthImageToPointCloud.h>
#include <vtkPlaneSource.h>
#include <vtkLight.h>
#include <vtkPointData.h>
#include <vtkCallbackCommand.h>

#include <utility>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

#include "../utilities/experiment_utils.h"
#include "../vtk/VtkRobotModel.h"
#include "../utilities/load_mesh.h"
#include "../utilities/moveit.h"
#include "../exploration/StupidAlgorithm.h"
#include "../exploration/ColorEncoding.h"
#include "../exploration/VtkToPointCloud.h"
#include "../utilities/vtk.h"
#include "../TreeMeshes.h"
#include "../ScannablePointsIndex.h"

struct SimplifiedOrchard {
	std::vector<std::pair<Eigen::Vector2d, TreeMeshes>> trees;
};

vtkNew<vtkActorCollection> buildOrchardActors(const SimplifiedOrchard &orchard);

vtkNew<vtkActor> buildGroundPlaneActor();

vtkNew<vtkRenderWindow> buildSensorRenderWindow(vtkNew<vtkRenderer> &sensorRenderer);

vtkNew<vtkRenderer> buildSensorRenderer();

vtkNew<vtkPolyData> mkVtkPoolyDataFromScannablePoints(const std::vector<ScanTargetPoint> &fruitSurfacePoints);

vtkNew<vtkActor> constructSimplePolyDataPointCloudActor(vtkNew<vtkPolyData> &fruitSurfacePolyData);

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

vtkNew<vtkRenderWindow> buildViewerWindow(vtkNew<vtkRenderer> &viewerRenderer) {
	vtkNew<vtkRenderWindow> visualizerWindow;
	visualizerWindow->SetSize(800,600);
	visualizerWindow->SetWindowName("PointCloud");
	visualizerWindow->AddRenderer(viewerRenderer);
	return visualizerWindow;
}

vtkNew<vtkRenderWindowInteractor> buildVisualizerWindowInteractor(vtkNew<vtkRenderWindow> &visualizerWindow) {
	vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
	renderWindowInteractor->SetRenderWindow(visualizerWindow);
	renderWindowInteractor->CreateRepeatingTimer(33);
	return renderWindowInteractor;
}

vtkNew<vtkActor> buildPointCloudActor(vtkNew<vtkDepthImageToPointCloud> &depthToPointCloud) {
	vtkNew<vtkPolyDataMapper> pointCloudMapper;
	pointCloudMapper->SetInputConnection(depthToPointCloud->GetOutputPort());

	vtkNew<vtkActor> pointCloudActor;
	pointCloudActor->SetMapper(pointCloudMapper);
	return pointCloudActor;
}

Eigen::Vector3d toEigen(const geometry_msgs::msg::Point &point) {
	return {point.x, point.y, point.z };
}

std::vector<ScanTargetPoint> buildScanTargetPoints(const shape_msgs::msg::Mesh &mesh, size_t n) {

	// Distribute 1000 points on the surface of the mesh.

	// TODO: Argue uniformity of some kind.

	std::vector<ScanTargetPoint> points;
	points.reserve(n);

	ompl::RNG rng;
	for (size_t i = 0; i < n; i++) {
		const auto &triangle = mesh.triangles[rng.uniformInt(0, (int) mesh.triangles.size() - 1)];

		const auto &p1 = toEigen(mesh.vertices[triangle.vertex_indices[0]]);
		const auto &p2 = toEigen(mesh.vertices[triangle.vertex_indices[1]]);
		const auto &p3 = toEigen(mesh.vertices[triangle.vertex_indices[2]]);

		const double r1 = rng.uniform01();
		const double r2 = rng.uniform01();

		Eigen::Vector3d sample = p1 + r1 * (p2 - p1) + r2 * (p3 - p1);

		ScanTargetPoint point;
		point.point = sample;
		points.push_back(point);
	}

	return points;

}

vtkNew<vtkActor> constructSimplePolyDataPointCloudActor(vtkNew<vtkPolyData> &fruitSurfacePolyData) {
	vtkNew<vtkActor> fruitSurfacePointsActor;
	vtkNew<vtkPolyDataMapper> fruitSurfacePointsMapper;
	fruitSurfacePointsMapper->SetInputData(fruitSurfacePolyData);

	fruitSurfacePointsActor->SetMapper(fruitSurfacePointsMapper);
	fruitSurfacePointsActor->GetProperty()->SetPointSize(20);
	return fruitSurfacePointsActor;
}

vtkNew<vtkPolyData> mkVtkPoolyDataFromScannablePoints(const std::vector<ScanTargetPoint> &fruitSurfacePoints) {
	vtkNew<vtkUnsignedCharArray> colors;
	colors->SetNumberOfComponents(3);

	vtkNew<vtkPoints> fruitSurfacePointsVtk;
	vtkNew<vtkCellArray> fruitSurfaceCells;

	for (const auto &point: fruitSurfacePoints) {
		auto pt_id = fruitSurfacePointsVtk->InsertNextPoint(point.point.data());
		fruitSurfaceCells->InsertNextCell({pt_id});

		colors->InsertNextTuple3(0, 0, 255);
	}

	vtkNew<vtkPolyData> fruitSurfacePolyData;
	fruitSurfacePolyData->SetPoints(fruitSurfacePointsVtk);
	fruitSurfacePolyData->SetVerts(fruitSurfaceCells);
	fruitSurfacePolyData->GetPointData()->SetScalars(colors);
	return fruitSurfacePolyData;
}

vtkNew<vtkRenderer> buildViewerRenderer() {
	vtkNew<vtkRenderer> viewerRenderer;
	viewerRenderer->SetBackground(0.1, 0.1, 0.5);
	viewerRenderer->ResetCamera();

	vtkNew<vtkLight> naturalLight;
	naturalLight->SetAmbientColor(0.0, 0.0, 0.0);
	viewerRenderer->ClearLights();
	viewerRenderer->AddLight(naturalLight);
	return viewerRenderer;
}


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

	auto surface_points = buildScanTargetPoints(orchard.trees[0].second.fruit_mesh, 2000);
	
	ScannablePointsIndex scannablePointsIndex(surface_points);

	vtkNew<vtkPolyData> fruitSurfacePolyData = mkVtkPoolyDataFromScannablePoints(surface_points);

	vtkNew<vtkActor> fruitSurfacePointsActor = constructSimplePolyDataPointCloudActor(fruitSurfacePolyData);

	auto orchard_actors = buildOrchardActors(orchard);

	vtkNew<vtkRenderer> sensorRenderer = buildSensorRenderer();
	vtkNew<vtkRenderWindow> sensorViewWindow = buildSensorRenderWindow(sensorRenderer);
	sensorViewWindow->Render();

	{
		addActorCollectionToRenderer(orchard_actors, sensorRenderer);
		addActorCollectionToRenderer(robotModel.getLinkActors(), sensorRenderer);
	}

	vtkNew<vtkDepthImageToPointCloud> depthToPointCloud = extractPointCloudFromRenderer(sensorRenderer);

	vtkNew<vtkCallbackCommand> pointCloudCallback;

	vtkNew<vtkActor> pointCloudActor = buildPointCloudActor(depthToPointCloud);

	vtkNew<vtkRenderer> viewerRenderer = buildViewerRenderer();

	{
		viewerRenderer->AddActor(pointCloudActor);
		addActorCollectionToRenderer(orchard_actors, viewerRenderer);
		addActorCollectionToRenderer(robotModel.getLinkActors(), viewerRenderer);
		viewerRenderer->AddActor(fruitSurfacePointsActor);
	}

	vtkNew<vtkRenderWindow> visualizerWindow = buildViewerWindow(viewerRenderer);
	vtkNew<vtkRenderWindowInteractor> renderWindowInteractor = buildVisualizerWindowInteractor(visualizerWindow);

	double pathProgressT = 0.0;
	std::optional<robot_trajectory::RobotTrajectory> currentTrajectory;

	StupidGoToFirstPointAlgorithm stupidAlgorithm([&](robot_trajectory::RobotTrajectory trajectory) {
		currentTrajectory = std::move(trajectory);
		pathProgressT = 0.0;
	});

	vtkNew<vtkFunctionalCallback> cb;
	cb->setEventId(vtkCommand::TimerEvent);
	cb->setCallback([&](){

		if (currentTrajectory) {

			double maxT = currentTrajectory->getWayPointDurationFromStart(
					currentTrajectory->getWayPointCount() - 1);

			pathProgressT += 0.033;

			if (pathProgressT > maxT) {
				pathProgressT = maxT;
			}

			{
				moveit::core::RobotStatePtr state_fakeshared(moveit::core::RobotStatePtr{}, &current_state);
				currentTrajectory->getStateAtDurationFromStart(pathProgressT, state_fakeshared);
			}

			current_state.update(true);

		}

		robotModel.applyState(current_state);
		Eigen::Isometry3d eePose = current_state.getGlobalLinkTransform("end_effector");
		setCameraFromEigen(eePose, sensorRenderer->GetActiveCamera());
		sensorViewWindow->Render();

		// Nullptr check to avoid a race condition where the callback is called before the first point cloud is rendered.
		// FIXME: Ideally, I'd like to make sure the algorithm sees the point cloud exactly once.
		if (depthToPointCloud->GetOutput()->GetPoints() != nullptr) {

			SegmentedPointCloud segmentedPointCloud = segmentPointCloudData(depthToPointCloud->GetOutput());

			stupidAlgorithm.updatePointCloud(current_state, segmentedPointCloud);

			auto scanned_points = scannablePointsIndex.findScannedPoints(
											   current_state.getGlobalLinkTransform("end_effector").translation(),
											   current_state.getGlobalLinkTransform("end_effector").rotation() * Eigen::Vector3d(0, 1, 0),
											   M_PI / 4.0,
											   0.1,
											   segmentedPointCloud
											   );

			for (auto &point: scanned_points) {
				std::cout << "Yes!" << std::endl;
				fruitSurfacePolyData->GetPointData()->GetScalars()->SetTuple3((vtkIdType) point, 255, 0, 255);
				fruitSurfacePolyData->Modified();
			}
		}

		visualizerWindow->Render();

	});

	renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, cb);
	renderWindowInteractor->AddObserver(vtkCommand::WindowFrameEvent, cb);

	renderWindowInteractor->Start();

	return EXIT_SUCCESS;
}


vtkNew<vtkRenderWindow> buildSensorRenderWindow(vtkNew<vtkRenderer> &sensorRenderer) {
	vtkNew<vtkRenderWindow> sensorViewWindow;
	sensorViewWindow->SetSize(200, 200);
	sensorViewWindow->AddRenderer(sensorRenderer);
	sensorViewWindow->SetWindowName("Robot Sensor View");
	return sensorViewWindow;
}

vtkNew<vtkRenderer> buildSensorRenderer() {
	vtkNew<vtkRenderer> sensorRenderer;
	sensorRenderer->SetBackground(0.0, 0.0, 0.0);
	sensorRenderer->GetActiveCamera()->SetClippingRange(0.1, 10.0);

	vtkNew<vtkLight> solidColorLight = mkWhiteAmbientLight();
	sensorRenderer->ClearLights();
	sensorRenderer->AddLight(solidColorLight);
	return sensorRenderer;
}

void setColorsByEncoding(vtkNew<vtkActor> &tree_actor, const std::array<double, 3> &rgb) {
	tree_actor->GetProperty()->SetDiffuseColor(rgb.data());
	tree_actor->GetProperty()->SetAmbientColor(rgb.data());
	tree_actor->GetProperty()->SetAmbient(1.0);
}


vtkNew<vtkActorCollection> buildTreeActors(const TreeMeshes& meshes) {

	vtkNew<vtkActorCollection> actors;

	auto tree_actor = createActorFromMesh(meshes.trunk_mesh);
	setColorsByEncoding(tree_actor, TRUNK_RGB);

	auto leaves_actor = createActorFromMesh(meshes.leaves_mesh);
	setColorsByEncoding(leaves_actor, LEAVES_RGB);

	auto fruit_actor = createActorFromMesh(meshes.fruit_mesh);
	setColorsByEncoding(fruit_actor, FRUIT_RGB);

	actors->AddItem(tree_actor);
	actors->AddItem(leaves_actor);
	actors->AddItem(fruit_actor);

	return actors;
}

vtkNew<vtkActorCollection> buildOrchardActors(const SimplifiedOrchard &orchard) {

	vtkNew<vtkActorCollection> orchard_actors;

	for (const auto& [pos, meshes] : orchard.trees)
	{
		auto tree_actors = buildTreeActors(meshes);

		for (int i = 0; i < tree_actors->GetNumberOfItems(); i++) {
			auto actor = vtkActor::SafeDownCast(tree_actors->GetItemAsObject(i));
			actor->SetPosition(pos.x(), pos.y(), 0);

			orchard_actors->AddItem(actor);

		}
	}

	vtkNew<vtkActor> ground_plane_actor = buildGroundPlaneActor();

	orchard_actors->AddItem(ground_plane_actor);

	return orchard_actors;

}

vtkNew<vtkActor> buildGroundPlaneActor() {
	vtkNew<vtkPlaneSource> ground_plane_source;
	ground_plane_source->SetPoint1(10.0, 0.0, 0.0);
	ground_plane_source->SetPoint2(0.0, 10.0, 0.0);
	ground_plane_source->SetCenter(0.0, 0.0, 0.0);

	vtkNew<vtkPolyDataMapper> ground_plane_mapper;
	ground_plane_mapper->SetInputConnection(ground_plane_source->GetOutputPort());

	vtkNew<vtkActor> ground_plane_actor;
	ground_plane_actor->SetMapper(ground_plane_mapper);
	ground_plane_actor->GetProperty()->SetDiffuseColor(GROUND_PLANE_RGB.data());
	ground_plane_actor->GetProperty()->SetAmbientColor(GROUND_PLANE_RGB.data());
	ground_plane_actor->GetProperty()->SetAmbient(1.0);
	return ground_plane_actor;
}
