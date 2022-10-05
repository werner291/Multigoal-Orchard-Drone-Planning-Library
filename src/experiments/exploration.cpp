
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

#include "../utilities/experiment_utils.h"
#include "../vtk/VtkRobotModel.h"
#include "../utilities/load_mesh.h"
#include "../utilities/moveit.h"
#include "../exploration/StupidAlgorithm.h"
#include "../exploration/ColorEncoding.h"
#include "../exploration/VtkToPointCloud.h"
#include "../utilities/vtk.h"

moveit::core::RobotState mkInitialState(const moveit::core::RobotModelPtr &drone);

vtkNew<vtkActor> buildDepthImagePointCloudActor(vtkNew<vtkDepthImageToPointCloud> &depthToPointCloud);

std::vector<ScanTargetPoint> buildScanTargetPoints(const shape_msgs::msg::Mesh &mesh, size_t n);

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

	vtkNew<vtkPolyData> fruitSurfacePolyData = mkVtkPolyDataFromScannablePoints(surface_points);

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

	vtkNew<vtkActor> pointCloudActor = buildDepthImagePointCloudActor(depthToPointCloud);

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



