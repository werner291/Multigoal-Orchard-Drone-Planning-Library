
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
#include "../exploration/SurfletVolume.h"
#include "../utilities/shape_generation.h"
#include "../MinimumClearanceOctree.h"

/**
 * Build a MoveIt collision environment from the given workspace specification.
 *
 * @param spec 			The workspace specification.
 *
 * @return 				The collision environment.
 */
std::unique_ptr<collision_detection::CollisionEnvFCL>
buildOrchardAndRobotFCLCollisionEnvironment(const WorkspaceSpec &spec);


WorkspaceSpec buildWorkspaceSpec() {

	auto drone = loadRobotModel(1.0);

	auto initial_state = mkInitialState(drone);

	setBaseOrientation(initial_state, Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ())));

	initial_state.update(true);

	return std::move<WorkspaceSpec>({drone, initial_state, {{{{0.0, 0.0}, loadTreeMeshes("appletree")}}}});
}

SurfletVolume visibleVolumeFromPointCloud(const std::vector<Eigen::Vector3d> &points,
										  const Eigen::Isometry3d &sensor_pose,
										  double max_range) {

	SurfletVolume volume1;

	// For testing purposes, generate a simple circle around the sensor in the XY plane.

	const size_t NUM_POINTS = 16;

	for (size_t i = 0; i < NUM_POINTS; ++i) {

		Eigen::Vector3d ray(cos(2.0 * M_PI * i / NUM_POINTS), sin(2.0 * M_PI * i / NUM_POINTS), 0.0);

		SurfletVolume::Surflet s;
		s.point = sensor_pose.translation() + ray * 0.5;
		s.normal = ray;

		volume1.add(s);
	}

	return volume1;

	//
	//	for (const auto &pt: spherifiedCubeVertices(3)) {
	//		SurfletVolume::Surflet s;
	//		s.point = pt * 0.5 + sensor_pose.translation();
	//		s.normal = pt;
	//		volume1.add(s);
	//	}
	//
	//	return volume1;

	struct PointWithDepth {
		Eigen::Vector2d point;
		double depth;
	};

	std::vector<PointWithDepth> points_with_depth;

	for (const auto &point: points) {
		Eigen::Vector3d point_in_sensor_frame = sensor_pose.inverse() * point;

		points_with_depth.push_back({{point_in_sensor_frame.x() / point_in_sensor_frame.y(), // Y is the depth axis
									  point_in_sensor_frame.z() / point_in_sensor_frame.y()},
									 point_in_sensor_frame.y()});

	}

	Eigen::AlignedBox2d bounding_box;

	for (const auto &point: points_with_depth) {
		bounding_box.extend(point.point);
	}

	size_t grid_size = 10;

	std::vector<double> depth_grid(grid_size * grid_size, std::numeric_limits<double>::infinity());

	for (const auto &point: points_with_depth) {

		Eigen::Vector2d grid_cell_size = (bounding_box.max() - bounding_box.min()) / ((double) grid_size - 1.0);

		Eigen::Vector2d point_in_grid = (point.point - bounding_box.min()).cwiseQuotient(grid_cell_size);

		size_t x = std::floor(point_in_grid.x());
		size_t y = std::floor(point_in_grid.y());

		assert(x < grid_size);
		assert(y < grid_size);

		depth_grid[x + y * grid_size] = std::min(depth_grid[x + y * grid_size], point.depth);
	}

	SurfletVolume volume;

	for (size_t x = 0; x < grid_size; x++) {
		for (size_t y = 0; y < grid_size; y++) {
			if (depth_grid[x + y * grid_size] < max_range) {

				Eigen::Vector3d point = sensor_pose * Eigen::Vector3d(bounding_box.min().x() + ((double) x) *
																							   (bounding_box.max().x() -
																								bounding_box.min()
																										.x()) /
																							   ((double) grid_size -
																								1.0),
																	  depth_grid[x + y * grid_size],
																	  bounding_box.min().y() + ((double) y) *
																							   (bounding_box.max().y() -
																								bounding_box.min()
																										.y()) /
																							   ((double) grid_size -
																								1.0));

				Eigen::Vector3d normal = -(sensor_pose.translation() - point).normalized();

				//				volume.add({point, normal});

				if (x == 0 || x == grid_size - 1 || y == 0 || y == grid_size - 1) {

					size_t pts = 5;//std::floor(depth_grid[x + y * grid_size] / 0.5);

					for (size_t i = 1; i < pts; i++) {

						double t = ((double) i) / ((double) pts - 1.0);

						Eigen::Vector3d interpolated = sensor_pose.translation() * (1.0 - t) + point * t;

						Eigen::Vector3d interpolated_normal;

						if (x == 0) {
							interpolated_normal = Eigen::Vector3d(-1.0, 0.0, 0.0);
						} else if (x == grid_size - 1) {
							interpolated_normal = Eigen::Vector3d(1.0, 0.0, 0.0);
						} else if (y == 0) {
							interpolated_normal = Eigen::Vector3d(0.0, 0.0, -1.0);
						} else if (y == grid_size - 1) {
							interpolated_normal = Eigen::Vector3d(0.0, 0.0, 1.0);
						}

						volume.add({interpolated, sensor_pose.rotation() * interpolated_normal});
					}

				}
			}


		}
	}

	// TODO Cap near the eye position

	return volume;

}

int main(int, char *[]) {

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

	{
		robot_trajectory::RobotTrajectory trajectory(workspaceSpec.robotModel);

		trajectory.addSuffixWayPoint(workspaceSpec.initialState, 0.0);

		moveit::core::RobotState rs2 = trajectory.getLastWayPoint();
		setBaseTranslation(rs2, getBaseTranslation(workspaceSpec.initialState) + Eigen::Vector3d(2.0, 0.0, 0.0));
		trajectory.addSuffixWayPoint(rs2, 1.0);

		moveit::core::RobotState rs3 = trajectory.getLastWayPoint();
		setBaseTranslation(rs3, getBaseTranslation(workspaceSpec.initialState) + Eigen::Vector3d(2.0, 2.0, 0.0));
		trajectory.addSuffixWayPoint(rs3, 2.0);

		currentPathState.newPath(trajectory);
	}

	//	// Initialize the motion-planning algorithm with the robot's initial state,
	//	// and a callback for when the algorithm has found a new path.
	//	DynamicMeshHullAlgorithm dbsa(workspaceSpec.initialState, [&](robot_trajectory::RobotTrajectory trajectory) {
	//
	//		// Simply hand it off to the current-path-state object; it'll take care of moving the robot.
	//		currentPathState.newPath(trajectory);
	//
	//		// Extract the end-effector trace from the trajectory and visualize it.
	//		viewer.ee_trace_visualization.updateLine(extractEndEffectorTrace(trajectory));
	//
	//	}, std::move(wrapper_algo));

	double time = 0.0;

	// Generate a log file name with the current date and time

	auto log_file = open_new_logfile(workspaceSpec.orchard.trees[0].second.fruit_meshes.size());

	MinimumClearanceOctree seen_space(Eigen::AlignedBox3d(Eigen::Vector3d(-10.0, -10.0, -10.0),
														  Eigen::Vector3d(10.0, 10.0, 10.0)), 0.2);

	size_t frames = 0;

	VtkPointCloudVisualization seen_space_visualization({1.0, 0.0, 1.0});

	viewer.addActor(seen_space_visualization.getActor());

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

		seen_space.update([&](const Eigen::Vector3d &p) -> std::pair<double, Eigen::Vector3d> {
			Eigen::Vector3d delta = eePose.translation() - p;

			return {1.0 - delta.norm(), -delta.normalized()};

		});

		seen_space_visualization.updatePoints(implicitSurfacePoints(seen_space));

		// Pass the point cloud to the motion-planning algorithm; it will probably respond by emitting a new path.
		//		dbsa.update(currentPathState.getCurrentState(), points);

		//		viewer.updateAlgorithmVisualization(dbsa);

		// Update the visualization of the scannable points, recoloring the ones that were just seen.
		//		fruitSurfaceScanTargetsActor.markAsScanned(new_scanned_points);

		if (frames++ % 100 == 0) {
			write_log_csv_line(log_file,
							   {.time = time, .per_fruit_scan_proportion = surface_scan_tracker.per_target_scan_portion()});
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





