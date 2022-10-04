
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
#include <vtkRendererSource.h>
#include <vtkPointData.h>

#include <utility>

#include "../utilities/experiment_utils.h"
#include "../vtk/VtkRobotModel.h"
#include "../utilities/load_mesh.h"
#include "../DroneStateConstraintSampler.h"
#include "../utilities/moveit.h"
#include "../exploration/StupidAlgorithm.h"
#include "../exploration/ColorEncoding.h"
#include "../exploration/VtkToPointCloud.h"
#include "../utilities/vtk.h"

vtkNew<vtkActorCollection> buildOrchardActors();

vtkNew<vtkActor> buildGroundPlaneActor();

vtkNew<vtkRenderWindow> buildSensorRenderWindow(vtkNew<vtkRenderer> &sensorRenderer);

vtkNew<vtkRenderer> buildSensorRenderer();

int main(int, char*[]) {

	auto drone = loadRobotModel();
	moveit::core::RobotState current_state(drone);
	current_state.setVariablePositions({
		5.0, 0.0, 1.5,
		0.0, 0.0, 0.0, 1.0,
		0.0, 0.0, 0.0, 0.0
	});
	current_state.update();

	VtkRobotmodel robotModel(drone);

	vtkNew<vtkActorCollection> orchard_actors = buildOrchardActors();

	vtkNew<vtkRenderer> sensorRenderer = buildSensorRenderer();
	vtkNew<vtkRenderWindow> sensorViewWindow = buildSensorRenderWindow(sensorRenderer);

	addActorCollectionToRenderer(orchard_actors, sensorRenderer);
	addActorCollectionToRenderer(robotModel.getLinkActors(), sensorRenderer);

	vtkNew<vtkDepthImageToPointCloud> depthToPointCloud = extractPointCloudFromRenderer(sensorRenderer);

	vtkNew<vtkPolyDataMapper> pointCloudMapper;
	pointCloudMapper->SetInputConnection(depthToPointCloud->GetOutputPort());

	vtkNew<vtkActor> pointCloudActor;
	pointCloudActor->SetMapper(pointCloudMapper);
	sensorViewWindow->Render();

	vtkNew<vtkRenderer> viewerRenderer;
	viewerRenderer->AddActor(pointCloudActor);
	viewerRenderer->SetBackground(0.0, 0.0, 0.8);
	viewerRenderer->ResetCamera();

	vtkNew<vtkLight> naturalLight;
	naturalLight->SetAmbientColor(0.0, 0.0, 0.0);
	viewerRenderer->ClearLights();
	viewerRenderer->AddLight(naturalLight);

	addActorCollectionToRenderer(orchard_actors, viewerRenderer);
	addActorCollectionToRenderer(robotModel.getLinkActors(), viewerRenderer);

	vtkNew<vtkRenderWindow> visualizerWindow;
	visualizerWindow->SetSize(800,600);
	visualizerWindow->SetWindowName("PointCloud");
	visualizerWindow->AddRenderer(viewerRenderer);

	double t = 0;

	vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
	renderWindowInteractor->SetRenderWindow(visualizerWindow);
	renderWindowInteractor->CreateRepeatingTimer(33);

	double pathProgressT = 0.0;
	std::optional<robot_trajectory::RobotTrajectory> currentTrajectory;

	StupidGoToFirstPointAlgorithm stupidAlgorithm([&](robot_trajectory::RobotTrajectory trajectory) {
		currentTrajectory = std::move(trajectory);
		pathProgressT = 0.0;
	});

	vtkNew<vtkTimerCallback> cb;
	cb->setCallback([&](){

		if (currentTrajectory) {

			double maxT = currentTrajectory->getWayPointDurationFromStart(currentTrajectory->getWayPointCount() - 1);

			pathProgressT += 0.033;

			if (pathProgressT > maxT) {
				pathProgressT = maxT;
			}

			{
				moveit::core::RobotStatePtr state_fakeshared(moveit::core::RobotStatePtr{}, &current_state);
				currentTrajectory->getStateAtDurationFromStart(pathProgressT, state_fakeshared);
			}

			current_state.update(true);
			robotModel.applyState(current_state);

			Eigen::Isometry3d eePose = current_state.getGlobalLinkTransform("end_effector");
			setCameraFromEigen(eePose, sensorRenderer->GetActiveCamera());
			sensorViewWindow->Render();
		}

		stupidAlgorithm.updatePointCloud(current_state, segmentPointCloudData(depthToPointCloud->GetOutput()));

	});

	renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, cb);

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


vtkNew<vtkActorCollection> buildOrchardActors() {

	vtkNew<vtkActorCollection> orchard_actors;

	for (auto pos : {Eigen::Vector2d(-2.5,-2.5), Eigen::Vector2d(2.5, 2.5)})
	{
		auto tree_actor = createActorFromMesh(loadMesh("appletree_trunk.dae"));
		tree_actor->GetProperty()->SetDiffuseColor(TRUNK_RGB.data());
		tree_actor->GetProperty()->SetAmbientColor(TRUNK_RGB.data());
		tree_actor->GetProperty()->SetAmbient(1.0);

		auto leaves_actor = createActorFromMesh(loadMesh("appletree_leaves.dae"));
		leaves_actor->GetProperty()->SetDiffuseColor(LEAVES_RGB.data());
		leaves_actor->GetProperty()->SetAmbientColor(LEAVES_RGB.data());
		leaves_actor->GetProperty()->SetAmbient(1.0);

		auto fruit_actor = createActorFromMesh(loadMesh("appletree_fruit.dae"));
		fruit_actor->GetProperty()->SetDiffuseColor(FRUIT_RGB.data());
		fruit_actor->GetProperty()->SetAmbientColor(FRUIT_RGB.data());
		fruit_actor->GetProperty()->SetAmbient(1.0);

		for (auto act : {tree_actor.Get(), leaves_actor.Get(), fruit_actor.Get()})
		{
			act->SetPosition(pos.x(), pos.y(), 0.0);
			orchard_actors->AddItem(act);
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
