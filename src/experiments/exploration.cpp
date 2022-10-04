
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

const std::array<double, 3> GROUND_PLANE_RGB = {0.3, 0.2, 0.1};
const std::array<double, 3> FRUIT_RGB = {1.0, 0.0, 0.0};
const std::array<double, 3> TRUNK_RGB = {0.5, 0.3, 0.1};
const std::array<double, 3> LEAVES_RGB = {0.0, 1.0, 0.0};

vtkNew<vtkPoints> meshVerticesToVtkPoints(const shape_msgs::msg::Mesh &mesh);

vtkNew<vtkCellArray> meshTrianglesToVtkCells(const shape_msgs::msg::Mesh &mesh);

vtkNew<vtkPolyData> rosMeshToVtkPolyData(const shape_msgs::msg::Mesh &mesh);

vtkNew<vtkActor> createActorFromMesh(const shape_msgs::msg::Mesh &mesh);

vtkNew<vtkActorCollection> buildOrchardActors();

vtkNew<vtkActor> buildGroundPlaneActor();

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

enum PointType {
	PT_OBSTACLE,
	PT_SOFT_OBSTACLE,
	PT_TARGET
};

struct Point {
	PointType type;
	Eigen::Vector3d position;
};

struct SegmentedPointCloud {
	std::vector<Point> points;
};

std::optional<PointType> pointTypeByColor(const Eigen::Vector3d &color) {

	bool is_ground = abs(color.x() - std::floor(255.0 * GROUND_PLANE_RGB[0]))
			+ abs(color.y() - std::floor(255.0 * GROUND_PLANE_RGB[1]))
			+ abs(color.z() - std::floor(255.0 * GROUND_PLANE_RGB[2])) < 1.0e-6;

	bool is_fruit = abs(color.x() - std::floor(255.0 * FRUIT_RGB[0]))
			+ abs(color.y() - std::floor(255.0 * FRUIT_RGB[1]))
			+ abs(color.z() - std::floor(255.0 * FRUIT_RGB[2])) < 1.0e-6;

	bool is_trunk = abs(color.x() - std::floor(255.0 * TRUNK_RGB[0]))
			+ abs(color.y() - std::floor(255.0 * TRUNK_RGB[1]))
			+ abs(color.z() - std::floor(255.0 * TRUNK_RGB[2])) < 1.0e-6;

	bool is_leaves = abs(color.x() - std::floor(255.0 * LEAVES_RGB[0]))
			+ abs(color.y() - std::floor(255.0 * LEAVES_RGB[1]))
			+ abs(color.z() - std::floor(255.0 * LEAVES_RGB[2])) < 1.0e-6;

	if (is_ground || is_trunk) {
		return {PT_OBSTACLE};
	} else if (is_leaves) {
		return {PT_SOFT_OBSTACLE};
	} else if (is_fruit) {
		return {PT_TARGET};
	} else {
		return {};
	}
}

SegmentedPointCloud segmentPointCloudData(vtkPolyData *pPolyData) {
	auto points = pPolyData->GetPoints();
	auto colors = pPolyData->GetPointData()->GetScalars();

	SegmentedPointCloud segmentedPointCloud;

	assert(points->GetNumberOfPoints() == colors->GetNumberOfTuples());
	assert(colors->GetNumberOfComponents() == 3);

	for (int i = 0; i < points->GetNumberOfPoints(); i++) {
		Point point;
		points->GetPoint(i, point.position.data());

		Eigen::Vector3d color;
		colors->GetTuple(i, color.data());

		auto ptType = pointTypeByColor(color);

		if (ptType.has_value()) {
			point.type = *ptType;
			segmentedPointCloud.points.push_back(point);
		}
	}

	return std::move(segmentedPointCloud);
}

class OnlinePointCloudAlgorithm {

protected:
	std::function<void(robot_trajectory::RobotTrajectory)> trajectoryCallback;
public:
	explicit OnlinePointCloudAlgorithm(std::function<void(robot_trajectory::RobotTrajectory)> trajectoryCallback)
			: trajectoryCallback(std::move(trajectoryCallback)) {
	}

public:
	virtual void updatePointCloud(const moveit::core::RobotState& current_state, const SegmentedPointCloud &segmentedPointCloud) = 0;

};

class StupidGoToFirstPointAlgorithm : public OnlinePointCloudAlgorithm {

	bool firstPointFound = false;

public:
	explicit StupidGoToFirstPointAlgorithm(std::function<void(robot_trajectory::RobotTrajectory)> trajectoryCallback)
			: OnlinePointCloudAlgorithm(std::move(trajectoryCallback)) {
	}

	void updatePointCloud(const moveit::core::RobotState& current_state, const SegmentedPointCloud &segmentedPointCloud) override {

		if (!firstPointFound) {

			for (const auto &point : segmentedPointCloud.points) {

				if (point.type == PT_TARGET) {

					firstPointFound = true;

					robot_trajectory::RobotTrajectory trajectory(current_state.getRobotModel(), "whole_body");
					trajectory.addSuffixWayPoint(current_state, 0.0);

					moveit::core::RobotState targetState(current_state);
					moveEndEffectorToGoal(targetState, 0.0, point.position);
					trajectory.addSuffixWayPoint(targetState, current_state.distance(targetState));

					trajectoryCallback(std::move(trajectory));

					std::cout << "Found!" << std::endl;

					return;

				}
			}
			{
				// Rotate in place

				moveit::core::RobotState targetState(current_state);

				Eigen::Quaterniond base_pose(targetState.getGlobalLinkTransform("base_link").rotation());
				Eigen::Quaterniond target_pose = base_pose * Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ());

				setBaseOrientation(targetState, target_pose);

				robot_trajectory::RobotTrajectory trajectory(current_state.getRobotModel(), "whole_body");
				trajectory.addSuffixWayPoint(current_state, 0.0);
				trajectory.addSuffixWayPoint(targetState, current_state.distance(targetState));

				trajectoryCallback(std::move(trajectory));
				return;
			}
		}
	}


};


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

	vtkNew<vtkRendererSource> rendererSource;
	rendererSource->DepthValuesOn();
	rendererSource->SetInput(sensorRenderer);

//	vtkNew<vtkWindowToImageFilter> filter;
//	filter->SetInput(rendererSource->GetOutput());
//	filter->SetInputBufferTypeToZBuffer();

	vtkNew<vtkDepthImageToPointCloud> depthToPointCloud;
	depthToPointCloud->SetInputConnection(0, rendererSource->GetOutputPort());
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
	Eigen::Vector3d eye_up = eePose.rotation() * Eigen::Vector3d(0, 0.0, 1.0);

	pCamera->SetPosition(eye_center.x(), eye_center.y(), eye_center.z());
	pCamera->SetFocalPoint(eye_focus.x(), eye_focus.y(), eye_focus.z());
	pCamera->SetViewUp(eye_up.x(), eye_up.y(), eye_up.z());
}

void addActorCollectionToRenderer(vtkNew<vtkActorCollection> &orchard_actors, vtkNew<vtkRenderer> &sensorRenderer) {
	for (int i = 0; i < orchard_actors->GetNumberOfItems(); i++) {
		sensorRenderer->AddActor(vtkActor::SafeDownCast(orchard_actors->GetItemAsObject(i)));
	}
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
