
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

#include "../utilities/experiment_utils.h"
#include "../vtk/VtkRobotModel.h"
#include "../utilities/load_mesh.h"
#include "../DroneStateConstraintSampler.h"
#include "../utilities/moveit.h"

vtkNew<vtkPoints> meshVerticesToVtkPoints(const shape_msgs::msg::Mesh &mesh);

vtkNew<vtkCellArray> meshTrianglesToVtkCells(const shape_msgs::msg::Mesh &mesh);

vtkNew<vtkPolyData> rosMeshToVtkPolyData(const shape_msgs::msg::Mesh &mesh);

vtkNew<vtkActor> createActorFromMesh(const shape_msgs::msg::Mesh &mesh);

vtkNew<vtkActorCollection> buildOrchardActors();

vtkNew<vtkActor> buildGroundPlaneActor();

robot_trajectory::RobotTrajectory mkTrajectory(const moveit::core::RobotModelPtr &drone);

void addActorCollectionToRenderer(vtkNew<vtkActorCollection> &orchard_actors, vtkNew<vtkRenderer> &sensorRenderer);

void setCameraFromEigen(Eigen::Isometry3d &eePose, vtkCamera *pCamera);

vtkNew<vtkLight> mkWhiteAmbientLight();

class vtkTimerCallback : public vtkCommand
{
	std::function<void()> callback;

public:
	vtkTimerCallback() = default;

	static vtkTimerCallback* New()
	{
		return new vtkTimerCallback;
	}

	virtual void Execute(vtkObject* caller, unsigned long eventId,
						 void* vtkNotUsed(callData))
	{
		if (vtkCommand::TimerEvent == eventId)
		{
			callback();

			auto* iren = dynamic_cast<vtkRenderWindowInteractor*>(caller);
			iren->GetRenderWindow()->Render();
		}
	}

	void setCallback(const std::function<void()> &callback) {
		vtkTimerCallback::callback = callback;
	}
};

int main(int, char*[]) {

	auto drone = loadRobotModel();

	VtkRobotmodel robotModel(drone);

	robot_trajectory::RobotTrajectory trajectory = mkTrajectory(drone);

	vtkNew<vtkActorCollection> orchard_actors = buildOrchardActors();

	// The renderer generates the image
	// which is then displayed on the render window.
	// It can be thought of as a scene to which the actor is added
	vtkNew<vtkRenderer> sensorRenderer;

	addActorCollectionToRenderer(orchard_actors, sensorRenderer);
	addActorCollectionToRenderer(robotModel.getLinkActors(), sensorRenderer);
	sensorRenderer->SetBackground(0.0, 0.0, 0.0);
	sensorRenderer->GetActiveCamera()->SetClippingRange(0.1, 10.0);

	vtkNew<vtkLight> solidColorLight = mkWhiteAmbientLight();
	sensorRenderer->ClearLights();
	sensorRenderer->AddLight(solidColorLight);

	vtkNew<vtkRenderWindow> sensorViewWindow;
	sensorViewWindow->SetSize(200, 200);
	sensorViewWindow->AddRenderer(sensorRenderer);
	sensorViewWindow->SetWindowName("Robot Sensor View");
	vtkNew<vtkRenderWindowInteractor> sensorWindowInteractor;
	sensorWindowInteractor->SetRenderWindow(sensorViewWindow);

	vtkNew<vtkWindowToImageFilter> filter;
	filter->SetInput(sensorViewWindow);
	filter->SetInputBufferTypeToZBuffer();

	vtkNew<vtkDepthImageToPointCloud> depthToPointCloud;
	depthToPointCloud->SetInputConnection(filter->GetOutputPort());
	depthToPointCloud->SetCamera(sensorRenderer->GetActiveCamera());

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

	vtkNew<vtkTimerCallback> cb;
	cb->setCallback([&](){

		t += 0.033;

		if (t > trajectory.getWayPointDurationFromStart(trajectory.getWayPointCount() - 1)) {
			t = 0;
		}

		auto state = std::make_shared<moveit::core::RobotState>(drone);
		trajectory.getStateAtDurationFromStart(t, state);
		state->update(true);
		robotModel.applyState(*state);

		Eigen::Isometry3d eePose = state->getGlobalLinkTransform("end_effector");
		setCameraFromEigen(eePose, sensorRenderer->GetActiveCamera());
		sensorViewWindow->Render();
		filter->Modified();

		auto points = depthToPointCloud->GetOutput()->GetPoints();

		enum PointType {
			PT_OBSTACLE,
			PT_SOFT_OBSTACLE,
			PT_TARGET
		};

		struct SegmentedPointCloud {
			std::vector<Eigen::Vector3d> points;
			std::vector<PointType> types;
		};

		SegmentedPointCloud segmentedPointCloud;

		for (int i = 0; i < points->GetNumberOfPoints(); i++) {
			double p[3];
			points->GetPoint(i, p);
			segmentedPointCloud.points.emplace_back(p[0], p[1], p[2]);
		}

		std::cout << "Number of points: " << points->GetNumberOfPoints() << std::endl;

	});

	renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, cb);

	renderWindowInteractor->Start();

	return EXIT_SUCCESS;
}

vtkNew<vtkLight> mkWhiteAmbientLight() {
	vtkNew<vtkLight> light;
	light->SetDiffuseColor(0.0,0.0,0.0);
	light->SetAmbientColor(1.0,1.0,1.0);
	light->SetLightTypeToSceneLight();
	return light;
}

void setCameraFromEigen(Eigen::Isometry3d &eePose, vtkCamera *pCamera) {
	Eigen::Vector3d eye_center = eePose.translation();
	Eigen::Vector3d eye_focus = eePose * Eigen::Vector3d(0, 1.0, 0.0);
	Eigen::Vector3d eye_up = eePose * Eigen::Vector3d(0, 0.0, 1.0);

	pCamera->SetPosition(eye_center.x(), eye_center.y(), eye_center.z());
	pCamera->SetFocalPoint(eye_focus.x(), eye_focus.y(), eye_focus.z());
	pCamera->SetViewUp(eye_up.x(), eye_up.y(), eye_up.z());
}

void addActorCollectionToRenderer(vtkNew<vtkActorCollection> &orchard_actors, vtkNew<vtkRenderer> &sensorRenderer) {
	for (int i = 0; i < orchard_actors->GetNumberOfItems(); i++) {
		sensorRenderer->AddActor(vtkActor::SafeDownCast(orchard_actors->GetItemAsObject(i)));
	}
}

robot_trajectory::RobotTrajectory mkTrajectory(const moveit::core::RobotModelPtr &drone) {
	robot_trajectory::RobotTrajectory trajectory(drone, "whole_body");

	moveit::core::RobotState robotState(drone);

	for (int i = 0; i <= 30; i++) {

		double t = i / 30.0;

		Eigen::Vector3d pos(cos(t * 2 * M_PI), sin(t * 2 * M_PI), 1.0);

		setBaseTranslation(robotState, pos);

		Eigen::Quaterniond q(Eigen::AngleAxisd(t * 2 * M_PI, Eigen::Vector3d::UnitZ()));

		setBaseOrientation(robotState, q);

		robotState.setVariablePosition(7, 0.0);
		robotState.setVariablePosition(8, 0.0);
		robotState.setVariablePosition(9, 0.0);
		robotState.setVariablePosition(10, 0.0);

		robotState.update(true);

		trajectory.addSuffixWayPoint(robotState, 1.0);

	}

	return std::move(trajectory);
}


vtkNew<vtkPolyData> rosMeshToVtkPolyData(const shape_msgs::msg::Mesh &mesh) {
	vtkNew<vtkPoints> points = meshVerticesToVtkPoints(mesh);

	vtkNew<vtkCellArray> cells = meshTrianglesToVtkCells(mesh);

	vtkNew<vtkPolyData> polyData;
	polyData->SetPoints(points);
	polyData->SetPolys(cells);

	return polyData;
}

vtkNew<vtkCellArray> meshTrianglesToVtkCells(const shape_msgs::msg::Mesh &mesh) {
	vtkNew<vtkCellArray> cells;
	for (auto &triangle : mesh.triangles) {
		cells->InsertNextCell({
			triangle.vertex_indices[0],
			triangle.vertex_indices[1],
			triangle.vertex_indices[2]
		});
	}
	return cells;
}

vtkNew<vtkPoints> meshVerticesToVtkPoints(const shape_msgs::msg::Mesh &mesh) {
	vtkNew<vtkPoints> points;
	for (auto &point : mesh.vertices) {
		points->InsertNextPoint(point.x, point.y, point.z);
	}
	return points;
}

vtkNew<vtkActor> createActorFromMesh(const shape_msgs::msg::Mesh &mesh) {

	vtkNew<vtkPolyData> polyData = rosMeshToVtkPolyData(mesh);

	vtkNew<vtkPolyDataMapper> mapper;
	mapper->SetInputData(polyData);

	vtkNew<vtkActor> actor;
	actor->SetMapper(mapper);

	return actor;
}

vtkNew<vtkActorCollection> buildOrchardActors() {

	vtkNew<vtkActorCollection> orchard_actors;

	for (auto pos : {Eigen::Vector2d(-2.5,-2.5), Eigen::Vector2d(2.5, 2.5)})
	{
		auto tree_actor = createActorFromMesh(loadMesh("appletree_trunk.dae"));
		tree_actor->GetProperty()->SetColor(0.4, 0.2, 0.1);
		tree_actor->GetProperty()->SetAmbientColor(0.4, 0.2, 0.1);
		tree_actor->GetProperty()->SetAmbient(1.0);

		auto leaves_actor = createActorFromMesh(loadMesh("appletree_leaves.dae"));
		leaves_actor->GetProperty()->SetColor(0.1, 0.6, 0.1);
		leaves_actor->GetProperty()->SetAmbientColor(0.1, 0.6, 0.1);
		leaves_actor->GetProperty()->SetAmbient(1.0);

		auto fruit_actor = createActorFromMesh(loadMesh("appletree_fruit.dae"));
		fruit_actor->GetProperty()->SetColor(1.0, 0.0, 0.0);
		fruit_actor->GetProperty()->SetAmbientColor(1.0, 0.0, 0.0);
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
	ground_plane_actor->GetProperty()->SetColor(0.5, 0.3, 0.1);
	ground_plane_actor->GetProperty()->SetAmbientColor(0.5, 0.3, 0.1);
	ground_plane_actor->GetProperty()->SetAmbient(1.0);
	return ground_plane_actor;
}
