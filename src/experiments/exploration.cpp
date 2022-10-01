
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

#include "../utilities/experiment_utils.h"
#include "../vtk/VtkRobotModel.h"
#include "../utilities/load_mesh.h"
#include "../DroneStateConstraintSampler.h"

vtkNew<vtkPoints> meshVerticesToVtkPoints(const shape_msgs::msg::Mesh &mesh);

vtkNew<vtkCellArray> meshTrianglesToVtkCells(const shape_msgs::msg::Mesh &mesh);

vtkNew<vtkPolyData> rosMeshToVtkPolyData(const shape_msgs::msg::Mesh &mesh);

vtkNew<vtkActor> createActorFromMesh(const shape_msgs::msg::Mesh &mesh);

vtkNew<vtkActorCollection> buildOrchardActors();

vtkNew<vtkActor> buildGroundPlaneActor();

robot_trajectory::RobotTrajectory mkTrajectory(const moveit::core::RobotModelPtr &drone);

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
	vtkNew<vtkRenderer> renderer;

	for (int i = 0; i < robotModel.getLinkActors()->GetNumberOfItems(); i++) {
		renderer->AddActor(vtkActor::SafeDownCast(robotModel.getLinkActors()->GetItemAsObject(i)));
	}

	for (int i = 0; i < orchard_actors->GetNumberOfItems(); i++) {
		renderer->AddActor(vtkActor::SafeDownCast(orchard_actors->GetItemAsObject(i)));
	}

	renderer->SetBackground(0.0,0.0,0.8);
	renderer->GetActiveCamera()->SetClippingRange(0.5, 10.0);
	renderer->ResetCamera();

	vtkNew<vtkRenderWindow> renderWindow;
	renderWindow->SetSize(200, 200);
	renderWindow->AddRenderer(renderer);
	renderWindow->SetWindowName("Robot Sensor View");
	vtkNew<vtkRenderWindowInteractor> sensorWindowInteractor;
	sensorWindowInteractor->SetRenderWindow(renderWindow);

	vtkNew<vtkWindowToImageFilter> filter;
	filter->SetInput(renderWindow);
	filter->SetInputBufferTypeToZBuffer();

	vtkNew<vtkDepthImageToPointCloud> depthToPointCloud;
	depthToPointCloud->SetInputConnection(filter->GetOutputPort());
	depthToPointCloud->SetCamera(renderer->GetActiveCamera());

	vtkNew<vtkPolyDataMapper> pointCloudMapper;
	pointCloudMapper->SetInputConnection(depthToPointCloud->GetOutputPort());

	vtkNew<vtkActor> pointCloudActor;
	pointCloudActor->SetMapper(pointCloudMapper);
	renderWindow->Render();

	vtkNew<vtkRenderer> pointCloudRenderer;
	pointCloudRenderer->AddActor(pointCloudActor);
	pointCloudRenderer->SetBackground(0.0,0.0,0.8);
	pointCloudRenderer->ResetCamera();

	for (int i = 0; i < robotModel.getLinkActors()->GetNumberOfItems(); i++)	{
		pointCloudRenderer->AddActor(vtkActor::SafeDownCast(robotModel.getLinkActors()->GetItemAsObject(i)));
	}

	for (int i = 0; i < orchard_actors->GetNumberOfItems(); i++) {
		pointCloudRenderer->AddActor(vtkActor::SafeDownCast(orchard_actors->GetItemAsObject(i)));
	}

	vtkNew<vtkRenderWindow> visualizerWindow;
	visualizerWindow->SetSize(800,600);
	visualizerWindow->SetWindowName("PointCloud");
	visualizerWindow->AddRenderer(pointCloudRenderer);

	vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
	renderWindowInteractor->SetRenderWindow(visualizerWindow);

	renderWindowInteractor->CreateRepeatingTimer(33);

	double t = 0;

	vtkNew<vtkTimerCallback> cb;

	cb->setCallback([&](){

		t += 0.5;

		if (t > trajectory.getWayPointDurationFromStart(trajectory.getWayPointCount() - 1)) {
			t = 0;
		}

		auto state = std::make_shared<moveit::core::RobotState>(drone);
		trajectory.getStateAtDurationFromStart(t, state);
		state->update(true);
		robotModel.applyState(*state);

		Eigen::Isometry3d eePose = state->getGlobalLinkTransform("end_effector");

		Eigen::Vector3d eye_center = eePose.translation();
		Eigen::Vector3d eye_focus = eePose * Eigen::Vector3d(0, 1.0, 0.0);
		Eigen::Vector3d eye_up = eePose * Eigen::Vector3d(0, 0.0, 1.0);

		renderer->GetActiveCamera()->SetPosition(eye_center.x(), eye_center.y(), eye_center.z());
		renderer->GetActiveCamera()->SetFocalPoint(eye_focus.x(), eye_focus.y(), eye_focus.z());
		renderer->GetActiveCamera()->SetViewUp(eye_up.x(), eye_up.y(), eye_up.z());

		renderWindow->Render();
		filter->Modified();

	});

	renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, cb);

	renderWindowInteractor->Start();

	return EXIT_SUCCESS;
}

robot_trajectory::RobotTrajectory mkTrajectory(const moveit::core::RobotModelPtr &drone) {
	robot_trajectory::RobotTrajectory trajectory(drone, "whole_body");

	moveit::core::RobotState robotState(drone);

	for (int i = 0; i <= 30; i++) {

		double t = i / 30.0;

		robotState.setVariablePosition(0, cos(t * 2 * M_PI));
		robotState.setVariablePosition(1, sin(t * 2 * M_PI));
		robotState.setVariablePosition(2, 2.0);

		Eigen::Quaterniond q(Eigen::AngleAxisd(t * 2 * M_PI, Eigen::Vector3d::UnitZ()));

		robotState.setVariablePosition(3, q.x());
		robotState.setVariablePosition(4, q.y());
		robotState.setVariablePosition(5, q.z());
		robotState.setVariablePosition(6, q.w());

		robotState.setVariablePosition(7, 0.0);
		robotState.setVariablePosition(8, 0.0);
		robotState.setVariablePosition(9, 0.0);
		robotState.setVariablePosition(10, 0.0);

		robotState.update(true);

		trajectory.addSuffixWayPoint(robotState, 10.0);

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

		auto leaves_actor = createActorFromMesh(loadMesh("appletree_leaves.dae"));
		leaves_actor->GetProperty()->SetColor(0.1, 0.6, 0.1);

		auto fruit_actor = createActorFromMesh(loadMesh("appletree_fruit.dae"));
		fruit_actor->GetProperty()->SetColor(1.0, 0.0, 0.0);

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
	return ground_plane_actor;
}
