
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkNew.h>
#include <vtkProperty.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPointData.h>
#include <vtkCallbackCommand.h>

#include <utility>

#include <range/v3/view/enumerate.hpp>

#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include <geometric_shapes/shape_operations.h>

#include "../utilities/experiment_utils.h"
#include "../vtk/VtkRobotModel.h"
#include "../utilities/load_mesh.h"
#include "../exploration/StupidAlgorithm.h"
#include "../exploration/ColorEncoding.h"
#include "../utilities/vtk.h"
#include "../vtk/SimulatedSensor.h"
#include "../vtk/Viewer.h"
#include "../exploration/DynamicBoundingSphereAlgorithm.h"
#include "../StreamingConvexHull.h"
#include "../WorkspaceSpec.h"
#include "../CurrentPathState.h"
#include "../utilities/moveit.h"
#include "../vtk/VisualizationSpecifics.h"

/**
 * Given a SimplifiedOrchard, create a SegmentedPointCloud that gives an initial hint about the contents
 * of the workspace, meant to be given initially to the exploration algorithm.
 *
 * It can be more or less detailed depending on how difficult you want the exploration to be.
 *
 * @param orchard 		The orchard to create the initial hint for.
 * @return 				The SegmentedPointCloud.
 */
SegmentedPointCloud generateInitialCloud(const SimplifiedOrchard &orchard);

/**
 * Build a MoveIt collision environment from the given workspace specification.
 *
 * @param spec 			The workspace specification.
 *
 * @return 				The collision environment.
 */
std::unique_ptr<collision_detection::CollisionEnvFCL> buildOrchardAndRobotFCLCollisionEnvironment(const WorkspaceSpec &spec);

/**
 * Get the leaf points from the given point cloud. It additionally filters out points that are too close to the ground.
 *
 * @param segmentedPointCloud 		The point cloud to get the leaf points from.
 * @return 							The leaf points.
 */
std::vector<Eigen::Vector3d> isolateLeafPoints(SegmentedPointCloud &segmentedPointCloud) {
	std::vector<Eigen::Vector3d> points;

	for (const auto &item: segmentedPointCloud.points) {
		if (item.type == SegmentedPointCloud::PT_SOFT_OBSTACLE && item.position.z() > 1.0e-6) {
			points.push_back(item.position);
		}
	}

	return points;
}

WorkspaceSpec buildWorkspaceSpec() {
	auto drone = loadRobotModel();
	return std::move<WorkspaceSpec>({
			drone,
			mkInitialState(drone),
			{{{ { 0.0, 0.0 }, loadTreeMeshes("appletree") }}}
	});
}

int main(int, char*[]) {

	WorkspaceSpec workspaceSpec = buildWorkspaceSpec();

	auto collision_env = buildOrchardAndRobotFCLCollisionEnvironment(workspaceSpec);

	auto surface_points = buildScanTargetPoints(workspaceSpec.orchard.trees[0].second.fruit_mesh, 2000);

	ScannablePointsIndex scannablePointsIndex(surface_points);

	FruitSurfaceScanTargetsActor fruitSurfaceScanTargetsActor(surface_points);

	VtkRobotmodel robotModel(workspaceSpec.robotModel,workspaceSpec.initialState);

	SimulatedSensor sensor = buildSensorSimulator(workspaceSpec.orchard, robotModel);

	vtkNew<vtkActor> pointCloudActor = buildDepthImagePointCloudActor(sensor.getPointCloudOutputPort());

	StreamingConvexHull convexHull = StreamingConvexHull::fromSpherifiedCube(3);

	ConvexHullActor convexHullActor;

	Viewer viewer = buildViewer(workspaceSpec.orchard, robotModel, fruitSurfaceScanTargetsActor.fruitSurfacePointsActor, pointCloudActor, convexHullActor.actor);

	CurrentPathState currentPathState(workspaceSpec.initialState);

	DynamicBoundingSphereAlgorithm dbsa([&](robot_trajectory::RobotTrajectory trajectory) {
		currentPathState.newPath(trajectory);
	});

	dbsa.updatePointCloud(currentPathState.current_state, generateInitialCloud(workspaceSpec.orchard));

	auto callback = [&]() {

		currentPathState.advance(0.005);

		if (checkCollision(currentPathState.getCurrentState(), *collision_env)) {
			std::cout << "Oh no!" << std::endl;
		}

		robotModel.applyState(currentPathState.getCurrentState());

		Eigen::Isometry3d eePose = currentPathState.getCurrentState().getGlobalLinkTransform("end_effector");
		sensor.requestRender(eePose);

		// Nullptr check to avoid a race condition where the callback is called before the first point cloud is rendered.
		// FIXME: Ideally, I'd like to make sure the algorithm sees the point cloud exactly once.
		if (auto points = sensor.extractLatestPointCloud()) {

			dbsa.updatePointCloud(currentPathState.getCurrentState(), *points);

			if (convexHull.addPoints(isolateLeafPoints(*points)))
			{
				convexHullActor.update(convexHull);
			}

			auto scanned_points = scannablePointsIndex
					.findScannedPoints(eePose.translation(),
									   eePose.rotation() * Eigen::Vector3d(0, 1, 0),
									   M_PI / 4.0,
									   0.1,
									   *points);

			fruitSurfaceScanTargetsActor.markAsScanned(scanned_points);
		}

		viewer.requestRender();

	};

	viewer.setIntervalCallback(callback);

	viewer.start();

	return EXIT_SUCCESS;
}

std::unique_ptr<collision_detection::CollisionEnvFCL> buildOrchardAndRobotFCLCollisionEnvironment(const WorkspaceSpec &workspaceSpec) {
	auto collision_world = std::make_shared<collision_detection::World>();

	for (const auto& [tree_id, tree] : workspaceSpec.orchard.trees | ranges::views::enumerate) {
		Eigen::Isometry3d tree_pose = Eigen::Isometry3d::Identity();
		tree_pose.translation() = Eigen::Vector3d(tree.first.x(), tree.first.y(), 0);
		collision_world->addToObject("tree_" + std::to_string(tree_id) + "_trunk",
									 shapes::ShapeConstPtr(shapes::constructShapeFromMsg(tree.second.trunk_mesh)),
									 tree_pose);
	}

	return std::move(std::make_unique<collision_detection::CollisionEnvFCL>(workspaceSpec.robotModel, collision_world));
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
