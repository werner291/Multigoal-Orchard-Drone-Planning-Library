
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkNew.h>
#include <vtkProperty.h>
#include <vtkPointData.h>
#include <vtkCallbackCommand.h>

#include <utility>

#include <range/v3/view/enumerate.hpp>

#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include <geometric_shapes/shape_operations.h>

#include "../exploration/logging.h"

#include "../utilities/experiment_utils.h"
#include "../vtk/VtkRobotModel.h"
#include "../utilities/load_mesh.h"
#include "../exploration/ColorEncoding.h"
#include "../utilities/vtk.h"
#include "../vtk/SimulatedSensor.h"
#include "../vtk/Viewer.h"

#include "../WorkspaceSpec.h"
#include "../CurrentPathState.h"
#include "../TriangleAABB.h"
#include "ScannedSurfaceTracker.h"

/**
 * Build a MoveIt collision environment from the given workspace specification.
 *
 * @param spec 			The workspace specification.
 *
 * @return 				The collision environment.
 */
std::unique_ptr<collision_detection::CollisionEnvFCL> buildOrchardAndRobotFCLCollisionEnvironment(const WorkspaceSpec &spec);


WorkspaceSpec buildWorkspaceSpec() {

	auto drone = loadRobotModel(1.0);

	auto initial_state = mkInitialState(drone);

	setBaseOrientation(initial_state, Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ())));

	initial_state.update(true);

	return std::move<WorkspaceSpec>({drone, initial_state,
			{{{ { 0.0, 0.0 }, loadTreeMeshes("appletree") }}}
	});
}

int main(int, char*[]) {

	// Build/load an abstract representation of the orchard that is as high-level and declarative as possible.
	// This includes models of the trees, the fruit within them, the robot, and the initial state of the robot.
	WorkspaceSpec workspaceSpec = buildWorkspaceSpec();

	// Build a collision environment that can be used to check for collisions between the robot and the orchard/environment.
	auto collision_env = buildOrchardAndRobotFCLCollisionEnvironment(workspaceSpec);

	PointSegmenter apple_surface_lookup(workspaceSpec);

	// Create a VTK actor to visualize the scannable points.
	//	FruitSurfaceScanTargetsActor fruitSurfaceScanTargetsActor(surface_points);`

	auto surface_scan_tracker = ScannedSurfaceTracker::buildForMeshes(workspaceSpec.orchard.trees[0].second
																			  .fruit_meshes);

	// Create a VTK actor to visualize the robot itself
	VtkRobotmodel robotModel(workspaceSpec.robotModel, workspaceSpec.initialState);

	// Create a simulated sensor that can be used to simulate the robot's camera; it can be used to extract point clouds visible to the robot.
	SimulatedSensor sensor = buildSensorSimulator(workspaceSpec.orchard, robotModel);

	// A VTK actor to visualize the point cloud extracted from the simulated sensor.
	vtkNew<vtkActor> pointCloudActor = buildDepthImagePointCloudActor(sensor.getPointCloudOutputPort());

	// A "package" of visualization-specifics that can be used to visualize the robot for debugging and presentation
	// purposes, SEPARATE from the visualization that simulates the depth sensor.
	Viewer viewer = Viewer(workspaceSpec.orchard,
						   robotModel,
						   (const std::vector<vtkActor *>) {//fruitSurfaceScanTargetsActor.fruitSurfacePointsActor,
								   pointCloudActor});

	// The "current" state of the robot based on the most recent-emitted path and progress of the robot along that path.
	CurrentPathState currentPathState(workspaceSpec.initialState);

	// The shell/wrapper algorithm to use as a parameter to the motion-planning algorithm
	auto wrapper_algo = std::make_shared<StreamingConvexHull>(StreamingConvexHull::fromSpherifiedCube(4));

	// Initialize the motion-planning algorithm with the robot's initial state,
	// and a callback for when the algorithm has found a new path.
	DynamicMeshHullAlgorithm dbsa(workspaceSpec.initialState, [&](robot_trajectory::RobotTrajectory trajectory) {

		// Simply hand it off to the current-path-state object; it'll take care of moving the robot.
		currentPathState.newPath(trajectory);

		// Extract the end-effector trace from the trajectory and visualize it.
		viewer.ee_trace_visualization.updateLine(extractEndEffectorTrace(trajectory));

	}, std::move(wrapper_algo));

	double time = 0.0;

	// Generate a log file name with the current date and time

	auto log_file = open_new_logfile(workspaceSpec.orchard.trees[0].second.fruit_meshes.size());


	size_t frames = 0;

	// The "main loop" (interval callback) of the program.
	auto callback = [&]() {

		// Advance the robot on the most recently-emitted path.
		currentPathState.advance(0.01);

		time += 0.01;

		// Check if the robot collided with anything.
		if (checkCollision(currentPathState.getCurrentState(), *collision_env)) {
			std::cout << "Collision!" << std::endl;
		}

		// Update the robot's visualization to match the current state.
		robotModel.applyState(currentPathState.getCurrentState());

		// Extract the end-effector pose from the robot state.
		Eigen::Isometry3d eePose = currentPathState.getCurrentState().getGlobalLinkTransform("end_effector");

		// Render the point cloud visible to the robot's camera.
		auto points = apple_surface_lookup.segmentPointCloudData(sensor.renderSnapshot(eePose));

		surface_scan_tracker.snapshot(eePose, points.target);

		// Pass the point cloud to the motion-planning algorithm; it will probably respond by emitting a new path.
		dbsa.update(currentPathState.getCurrentState(), points);

		viewer.updateAlgorithmVisualization(dbsa);

		// Update the visualization of the scannable points, recoloring the ones that were just seen.
		//		fruitSurfaceScanTargetsActor.markAsScanned(new_scanned_points);

		if (frames++ % 100 == 0) {
			write_log_csv_line(log_file,
							   {.time = time, .per_fruit_scan_proportion = surface_scan_tracker.per_target_scan_portion();});
		}

		viewer.requestRender();

	};

	// Set our "main loop" callback to be called every frame.
	viewer.setIntervalCallback(callback);

	// Run the app until termination.
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





