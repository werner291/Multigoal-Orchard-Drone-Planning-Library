// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 2/12/24.
//

#include <vector>
#include <random_numbers/random_numbers.h>

#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/VtkLineSegmentVizualization.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/surface_points.h"
#include "../experiment_utils/mesh_utils.h"
#include "../visualization/VtkPolyLineVisualization.h"
#include "../visualization/visualization_function_macros.h"
#include "../visualization/robot_state.h"
#include "../experiment_utils/scan_path_generators.h"
#include "../visualization/scannable_points.h"
#include "../planning/RobotPath.h"
#include "../planning/probing_motions.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../planning/state_tools.h"
#include "../planning/approach_path_planning.h"
#include "../experiment_utils/fcl_utils.h"

#include <fcl/narrowphase/collision.h>
#include <vtkTextActor.h>
#include <vtkProperty2D.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkSliderWidget.h>
#include <vtkSliderRepresentation2D.h>

#include "../planning/collision_detection.h"
#include "../planning/goal_sampling.h"
#include "../planning/scanning_motions.h"
#include "../planning/shell_path_assembly.h"
#include "../planning/spherical_geometry.h"
#include "../planning/shell_path.h"
#include "../planning/visitation_order.h"
#include "../experiment_utils/prompting.h"
#include "../experiment_utils/leaf_scaling.h"
#include "../visualization/VtkFunctionalCallback.h"

using namespace mgodpl;

REGISTER_VISUALIZATION(fruit_scan_fullpath) {
	// Load the tree meshes
	auto tree_model = tree_meshes::loadTreeMeshes("appletree");

	// Initialize a random number generator
	random_numbers::RandomNumberGenerator rng;

	// Define constants for the scannable points
	const size_t NUM_POINTS = 200;
	const double MAX_DISTANCE = 0.3;
	const double MIN_DISTANCE = 0;
	const double MAX_ANGLE = M_PI / 3.0;

	std::vector<ScannablePoints> all_scannable_points;
	std::vector<VtkLineSegmentsVisualization> fruit_points_visualizations;

	viewer.addMesh(tree_model.trunk_mesh, {0.5, 0.3, 0.1}, 1.0);

	// Iterate over all the fruit meshes in the tree model
	for (const auto &fruit_mesh: tree_model.fruit_meshes) {
		// Create the scannable points
		ScannablePoints scannable_points = createScannablePoints(
				rng,
				fruit_mesh, NUM_POINTS, MAX_DISTANCE, MIN_DISTANCE,
				MAX_ANGLE);

		// Create the fruit points visualization
		VtkLineSegmentsVisualization fruit_points_visualization = createFruitLinesVisualization(scannable_points);

		// Add the fruit points visualization to the viewer
		viewer.addActor(fruit_points_visualization.getActor());

		// Add the fruit mesh to the viewer
		viewer.addMesh(fruit_mesh, {0.8, 0.8, 0.8}, 1.0);

		fruit_points_visualizations.push_back(std::move(fruit_points_visualization));

		// Add the created scannable points to the vector
		all_scannable_points.push_back(std::move(scannable_points));

	}

	// Set the camera transform for the viewer
	viewer.setCameraTransform({2.0, 1.0, 1.0}, {0.0, 0.0, 0.0});

	const auto &robot = experiments::createProceduralRobotModel();

	robot_model::RobotModel::LinkId flying_base = robot.findLinkByName("flying_base");

	// Create a state outside the tree model.
	RobotState initial_state = fromEndEffectorAndVector(robot, {0, 5, 5}, {0, 1, 1});

	std::vector<SeenPoints> ever_seen;
	ever_seen.reserve(all_scannable_points.size());
	for (const auto &scannable_points: all_scannable_points) {
		ever_seen.push_back(SeenPoints::create_all_unseen(scannable_points));
	}

	// Plan the final path as a whole:
	RobotPath final_path = plan_multigoal_path(robot, tree_model, initial_state);

	PathPoint path_point = {0, 0.0};

	// Create an instance of VtkPolyLineVisualization
	VtkPolyLineVisualization end_effector_positions_visualization(1, 0.5, 1); // Red color

	// Add the polyline visualization to the viewer
	viewer.addActor(end_effector_positions_visualization.getActor());

	// Declare a vector to store the end-effector positions
	std::vector<mgodpl::math::Vec3d> end_effector_positions;

	auto robot_viz = mgodpl::vizualisation::vizualize_robot_state(viewer, robot,
																  robot_model::forwardKinematics(
																		  robot, initial_state.joint_values,
																		  flying_base, initial_state.base_tf));

	auto timerCallback = [&]() {
		const auto &start_state = final_path.states[path_point.segment_i];
		const auto &end_state = final_path.states[path_point.segment_i + 1];

		// Segment length:
		double segment_length = equal_weights_max_distance(start_state, end_state);

		// Advance the path point
		bool finished = advancePathPointClamp(final_path, path_point, 0.01, equal_weights_max_distance);

		auto interpolated_state = interpolate(start_state, end_state, path_point.segment_t);

		auto fk = robot_model::forwardKinematics(robot, interpolated_state.joint_values,
												 robot.findLinkByName("flying_base"), interpolated_state.base_tf);
		const math::Vec3d ee_pos = fk.forLink(robot.findLinkByName("end_effector")).translation;

		// Add the new end-effector position to the vector
		end_effector_positions.push_back(ee_pos);

		// Update the polyline with the new set of end-effector positions
		end_effector_positions_visualization.updateLine(end_effector_positions);

		// Update the robot's state in the visualization
		update_robot_state(robot, fk, robot_viz);

		size_t newly_seen = 0;

		for (size_t fruit_i = 0; fruit_i < all_scannable_points.size(); ++fruit_i) {
			newly_seen += update_visibility(all_scannable_points[fruit_i], ee_pos, ever_seen[fruit_i]);

			// Update the colors of the fruit points visualization
			fruit_points_visualizations[fruit_i].setColors(generateVisualizationColors(ever_seen[fruit_i]));
		}

		if (finished) {
			viewer.stop();

			size_t points_seen = 0;
			size_t points_total = 0;
			for (size_t fruit_i = 0; fruit_i < all_scannable_points.size(); ++fruit_i) {
				points_seen += ever_seen[fruit_i].count_seen();
				points_total += all_scannable_points[fruit_i].surface_points.size();
			}
			std::cout << "Seen " << points_seen << " out of " << points_total << " points" << std::endl;
		}
	};

	viewer.setCameraTransform({5.0, 5.0, 2.0}, {0.0, 0.0, 2.5});

	viewer.addTimerCallback(timerCallback);
	viewer.start();
}

REGISTER_VISUALIZATION(right_left_scanning_motion) {
	// Load the tree meshes
	auto tree_model = tree_meshes::loadTreeMeshes("appletree");

	// Initialize a random number generator
	random_numbers::RandomNumberGenerator rng(42);

	// Randomly select one fruit mesh
	int random_index = rng.uniformInteger(0, tree_model.fruit_meshes.size() - 1);
	auto &fruit_mesh = tree_model.fruit_meshes[random_index];

	// Define constants for the scannable points
	const size_t NUM_POINTS = 200;
	const double MAX_DISTANCE = 0.3;
	const double MIN_DISTANCE = 0;
	const double MAX_ANGLE = M_PI / 3.0;

	// Create the scannable points
	ScannablePoints scannable_points = createScannablePoints(
			rng,
			fruit_mesh, NUM_POINTS, MAX_DISTANCE, MIN_DISTANCE,
			MAX_ANGLE);

	// Create a robot model
	const auto &robot = experiments::createProceduralRobotModel();
	const robot_model::RobotModel::LinkId flying_base = robot.findLinkByName("flying_base");
	const robot_model::RobotModel::LinkId end_effector = robot.findLinkByName("end_effector");

	math::Vec3d fruit_center = mesh_aabb(fruit_mesh).center();

	// Allocate a BVH convex_hull for the tree trunk.
	const auto &tree_trunk_bvh = fcl_utils::meshToFclBVH(tree_model.trunk_mesh);
	fcl::CollisionObjectd tree_trunk_object(tree_trunk_bvh);

	// First, create the convex hull.
	cgal::CgalMeshData mesh_data(tree_model.leaves_mesh);

	const double EE_SCAN_DISTANCE = 0.1;

	std::optional<RobotState> sample = generateUniformRandomArmVectorState(
			robot,
			tree_trunk_object,
			fruit_center,
			rng,
			1000,
			EE_SCAN_DISTANCE
	);

	if (!sample.has_value()) {
		throw std::runtime_error("Failed to find a collision-free state");
	}

	// Get the arm vector from the sample:
	auto sample_fk = forwardKinematics(robot, sample->joint_values, flying_base, sample->base_tf);
	auto fruit_to_ee = sample_fk.forLink(end_effector).translation - fruit_center;

	// Get the lat/long angles of the arm vector:
	double lat_angle = spherical_geometry::latitude(fruit_to_ee);
	double long_angle = spherical_geometry::longitude(fruit_to_ee);

	// Then, try the straight-out motion:
	auto approach_path = straightout(robot, *sample, mesh_data.tree, mesh_data.mesh_path);

	PathPoint collision_point{};
	if (check_path_collides(robot, tree_trunk_object, approach_path.path, collision_point)) {
		throw std::runtime_error("Straight-out motion collides");
	}

	RobotPath path = approach_path.path;

	RobotPath sideways_scan = createLeftRightScanningMotion(
			robot,
			tree_trunk_object,
			fruit_center,
			long_angle,
			lat_angle,
			EE_SCAN_DISTANCE
	);

	path.states.insert(path.states.end(),
					   sideways_scan.states.begin(),
					   sideways_scan.states.end());

	// Define the current position on the path
	PathPoint path_point = {0, 0.0};

	// Define the speed of interpolation
	double interpolation_speed = 0.02;

	RobotState initial_state = path.states[0];

	// Create the fruit points visualization
	VtkLineSegmentsVisualization fruit_points_visualization = createFruitLinesVisualization(scannable_points);

	// Add the fruit points visualization to the viewer
	viewer.addActor(fruit_points_visualization.getActor());

	// Add the fruit mesh to the viewer
	viewer.addMesh(fruit_mesh, {0.8, 0.8, 0.8}, 1.0);

	// Add the tree trunk mesh to the viewer
	viewer.addMesh(tree_model.trunk_mesh, {0.5, 0.3, 0.1}, 1.0);

	// Set the camera transform for the viewer
	viewer.setCameraTransform({2.0, 1.0, 1.0}, {0.0, 0.0, 0.0});

	// Visualize the robot state
	auto robot_viz = vizualisation::vizualize_robot_state(viewer, robot,
														  forwardKinematics(
																  robot, initial_state.joint_values,
																  flying_base, initial_state.base_tf));

	// Initialize a vector to keep track of which points have been seen
	SeenPoints points_seen = SeenPoints::create_all_unseen(scannable_points);

	// Register the timer callback function to be called at regular intervals
	viewer.addTimerCallback([&]() {
		// Advance the path point along the path
		if (advancePathPointClamp(path, path_point, interpolation_speed, equal_weights_max_distance)) {
			viewer.stop();
		}

		// Interpolate the robot's state
		auto interpolated_state = interpolate(path_point, path);

		// Update the robot's state in the visualization
		const auto fk = forwardKinematics(robot, interpolated_state.joint_values,
										  robot.findLinkByName("flying_base"), interpolated_state.base_tf);

		update_robot_state(robot, fk, robot_viz);

		// Get the position of the robot's end effector
		const auto &end_effector_position = fk.forLink(robot.findLinkByName("end_effector")).translation;

		update_visibility(scannable_points, end_effector_position, points_seen);

		// Update the colors of the fruit points visualization
		fruit_points_visualization.setColors(generateVisualizationColors(points_seen));
	});

	viewer.setCameraTransform(fruit_center + math::Vec3d{2.5, 0.0, -1.5}, fruit_center);

	// Start the viewer
	viewer.start();
}

/**
 * @brief This function calculates the vector from the fruit to the end effector based on the last approach state.
 * It uses the forward kinematics of the robot to get the position of the end effector and then subtracts the fruit center position from it.
 *
 * @param robot The robot model.
 * @param fruit_center The center position of the fruit.
 * @param last_approach_state The last approach state of the robot.
 * @return math::Vec3d The vector from the fruit to the end effector.
 */
math::Vec3d calculateFruitToEEVector(const robot_model::RobotModel &robot,
									 const math::Vec3d &fruit_center,
									 const RobotState &last_approach_state) {
	const auto flying_base = robot.findLinkByName("flying_base");
	const auto fk = forwardKinematics(robot, last_approach_state.joint_values, flying_base,
									  last_approach_state.base_tf);
	const math::Vec3d ee_pos = fk.forLink(robot.findLinkByName("end_effector")).translation;
	const math::Vec3d fruit_to_ee = ee_pos - fruit_center;
	return fruit_to_ee;
}

REGISTER_VISUALIZATION(right_left_scanning_motion_all_apples) {
	// Load the tree meshes
	auto tree_model = tree_meshes::loadTreeMeshes("appletree");

	// Initialize a random number generator
	random_numbers::RandomNumberGenerator rng(42);

	// Define constants for the scannable points
	const size_t NUM_POINTS = 200;
	const double MAX_DISTANCE = 0.3;
	const double MIN_DISTANCE = 0;
	const double MAX_ANGLE = M_PI / 3.0;

	// Create the scannable points for all the fruit meshes
	std::vector<ScannablePoints> all_scannable_points;

	for (const auto &fruit_mesh: tree_model.fruit_meshes) {
		all_scannable_points.push_back(createScannablePoints(
				rng,
				fruit_mesh, NUM_POINTS, MAX_DISTANCE, MIN_DISTANCE,
				MAX_ANGLE));
	}

	// Create a robot model
	const auto &robot = experiments::createProceduralRobotModel();
	const robot_model::RobotModel::LinkId flying_base = robot.findLinkByName("flying_base");

	// Allocate a BVH convex_hull for the tree trunk.
	const auto &tree_trunk_bvh = fcl_utils::meshToFclBVH(tree_model.trunk_mesh);
	fcl::CollisionObjectd tree_trunk_object(tree_trunk_bvh);

	// First, create the convex hull.
	cgal::CgalMeshData mesh_data(tree_model.leaves_mesh);

	const double EE_SCAN_DISTANCE = 0.1;

	// First, create the initial approach path:
	ApproachPath initial_approach_path = approach_planning::plan_initial_approach_path(robot,
																					   fromEndEffectorAndVector(robot,
																												{0, 5,
																												 5},
																												{0, 1,
																												 1}),
																					   flying_base,
																					   mesh_data);

	// Try to plan an approach path for all the fruit meshes:
	std::vector<ApproachPath> approach_paths;
	std::vector<math::Vec3d> reachable_fruit_positions;

	size_t targets_planned_to = 0;
	for (const auto &tgt: computeFruitPositions(tree_model)) {
		auto straightout = uniform_straightout_approach(tgt, experiments::createProceduralRobotModel(),
														tree_trunk_object, mesh_data, rng, 1000, EE_SCAN_DISTANCE);

		if (straightout.has_value()) {
			approach_paths.push_back(*straightout);
			reachable_fruit_positions.push_back(tgt);
		}

		targets_planned_to++;

		std::cout << "Seen " << targets_planned_to << " out of " << tree_model.fruit_meshes.size() << " targets" <<
				  std::endl;
	}

	RobotState initial_state = fromEndEffectorAndVector(robot, {0, 5, 5}, {0, 1, 1});

	// compute the shell distances:
	const auto &initial_state_distances = shell_distances(initial_approach_path.shell_point,
														  approach_paths,
														  mesh_data.convex_hull);
	const auto &target_to_target_distances = shell_distances(approach_paths, mesh_data);

	const std::vector<size_t> &order = visitation_order_greedy(target_to_target_distances, initial_state_distances);

	RobotPath final_path;

	// Append the probe-retreat-move to the first goal.
	final_path.append(shell_path_planning::retreat_move_probe(robot, mesh_data.convex_hull,
															  initial_approach_path,
															  approach_paths[order[0]]));

	const math::Vec3d fruit_to_ee = calculateFruitToEEVector(robot, reachable_fruit_positions[order[0]], initial_state);

	// Then, the scanning motion:
	final_path.append(createLeftRightScanningMotion(
			robot,
			tree_trunk_object,
			reachable_fruit_positions[order[0]],
			spherical_geometry::longitude(fruit_to_ee),
			spherical_geometry::latitude(fruit_to_ee),
			EE_SCAN_DISTANCE
	));

	for (size_t i = 1; i < approach_paths.size(); ++i) {
		// Use retreat_move_probe to replace the selected code:

		final_path.append(shell_path_planning::retreat_move_probe(robot, mesh_data.convex_hull,
																  approach_paths[order[i - 1]],
																  approach_paths[order[i]]));

		const math::Vec3d fruit_to_ee = calculateFruitToEEVector(robot, reachable_fruit_positions[order[i]],
																 initial_state);

		// Then, the scanning motion:
		final_path.append(createLeftRightScanningMotion(
				robot,
				tree_trunk_object,
				reachable_fruit_positions[order[i]],
				spherical_geometry::longitude(fruit_to_ee),
				spherical_geometry::latitude(fruit_to_ee),
				EE_SCAN_DISTANCE
		));
	}

	// Define the current position on the path
	PathPoint path_point = {0, 0.0};

	// Define the speed of interpolation
	double interpolation_speed = 0.1;

	// Create the fruit points visualization
	std::vector<VtkLineSegmentsVisualization> fruit_points_visualizations;

	for (const auto &scannable_points: all_scannable_points) {
		fruit_points_visualizations.push_back(createFruitLinesVisualization(scannable_points));
		viewer.addActor(fruit_points_visualizations.back().getActor());
	}

	// Add the fruit mesh to the viewer
	for (const auto &fruit_mesh: tree_model.fruit_meshes) {
		viewer.addMesh(fruit_mesh, {0.8, 0.8, 0.8}, 1.0);
	}

	// Add the tree trunk mesh to the viewer
	viewer.addMesh(tree_model.trunk_mesh, {0.5, 0.3, 0.1}, 1.0);

	// Set the camera transform for the viewer
	viewer.setCameraTransform({2.0, 1.0, 1.0}, {0.0, 0.0, 0.0});

	// Visualize the robot state
	auto robot_viz = vizualisation::vizualize_robot_state(viewer, robot,
														  forwardKinematics(
																  robot, initial_state.joint_values,
																  flying_base, initial_state.base_tf));

	std::vector<SeenPoints> ever_seen;
	ever_seen.reserve(all_scannable_points.size());
	for (const auto &scannable_points: all_scannable_points) {
		ever_seen.push_back(SeenPoints::create_all_unseen(scannable_points));
	}

	// Start time:
	auto start_time = std::chrono::high_resolution_clock::now();

	// Register the timer callback function to be called at regular intervals
	viewer.addTimerCallback([&]() {
		// Advance the path point along the path
		if (advancePathPointClamp(final_path, path_point, interpolation_speed, equal_weights_max_distance)) {
			viewer.stop();
		}

		// Interpolate the robot's state
		auto interpolated_state = interpolate(path_point, final_path);

		// Update the robot's state in the visualization
		const auto fk = forwardKinematics(robot, interpolated_state.joint_values,
										  robot.findLinkByName("flying_base"), interpolated_state.base_tf);

		update_robot_state(robot, fk, robot_viz);

		// Get the position of the robot's end effector
		const auto &end_effector_position = fk.forLink(robot.findLinkByName("end_effector")).translation;

		for (size_t fruit_i = 0; fruit_i < all_scannable_points.size(); ++fruit_i) {
			update_visibility(all_scannable_points[fruit_i], end_effector_position, ever_seen[fruit_i]);
			fruit_points_visualizations[fruit_i].setColors(generateVisualizationColors(ever_seen[fruit_i]));
		}

		// Stop after 1 minute:
		auto current_time = std::chrono::high_resolution_clock::now();
		if (std::chrono::duration_cast<std::chrono::minutes>(current_time - start_time).count() > 1) {
			viewer.stop();
		}
	});

	viewer.setCameraTransform({5.0, 5.0, 3.5}, {0.0, 0.0, 2.5});

	// Start the viewer
	viewer.start();
}

REGISTER_VISUALIZATION(orbit_tree) {
	// Load the tree meshes
	auto tree_model = tree_meshes::loadTreeMeshes("appletree");

	// Initialize a random number generator
	random_numbers::RandomNumberGenerator rng;

	// Create a robot model
	const auto &robot = experiments::createProceduralRobotModel();
	const robot_model::RobotModel::LinkId flying_base = robot.findLinkByName("flying_base");

	// Define the center of the tree
	const auto leaves_aabb = mesh_aabb(tree_model.leaves_mesh);
	math::Vec3d tree_center = leaves_aabb.center();
	double tree_radius = leaves_aabb.size().norm() / 2.0;

	// Get the first JsonMeta<ParametricPath> object
	ParametricPath first_orbit = getSingleOrbit(tree_center, tree_radius);

	// Create a RobotPath from these states
	RobotPath path;

	// Define the number of steps to discretize the path
	size_t num_steps = 100;  // Adjust as needed

	// Generate a sequence of time values between 0 and 1
	for (size_t i = 0; i <= num_steps; ++i) {
		double t = static_cast<double>(i) / static_cast<double>(num_steps);

		// Call the ParametricPath function to get the position at time t
		math::Vec3d position = first_orbit(t);

		// Calculate the relative vector from the tree center
		math::Vec3d relative_vector = position - tree_center;

		// Add the state to the path
		path.states.push_back(fromEndEffectorAndVector(robot, position, relative_vector));
	}

	// Visualize the robot's path
	auto robot_viz = vizualisation::vizualize_robot_state(
			viewer,
			robot,
			forwardKinematics(
					robot, path.states[0].joint_values,
					flying_base,
					path.states[0].base_tf
			)
	);

	// Define the current position on the path
	PathPoint path_point = {0, 0.0};

	// Define the speed of interpolation
	double interpolation_speed = 0.01;

	// Add the tree trunk mesh to the viewer
	viewer.addMesh(tree_model.trunk_mesh, {0.5, 0.3, 0.1}, 1.0);

	// Define constants for the scannable points
	const size_t NUM_POINTS = 200;
	const double MAX_DISTANCE = INFINITY;
	const double MIN_DISTANCE = 0;
	const double MAX_ANGLE = M_PI / 3.0;

	PROMPT_USER_VALIDATED(double, leaf_scale, "Size of the leaves, where 1 is normal size: ", leaf_scale > 0.0)

	auto leaves = scale_leaves(tree_model, leaf_root_points(tree_model), leaf_scale);

	viewer.addMesh(leaves, {0.1, 0.5, 0.1}, 1.0);

	auto mesh_occlusion_model = std::make_shared<MeshOcclusionModel>(tree_model.leaves_mesh, 0.0);

	// Create the scannable points
	std::vector<ScannablePoints> all_scannable_points;
	for (const auto &fruit_mesh: tree_model.fruit_meshes) {
		all_scannable_points.push_back(createScannablePoints(
				rng,
				fruit_mesh,
				NUM_POINTS,
				MAX_DISTANCE,
				MIN_DISTANCE,
				MAX_ANGLE,
				mesh_occlusion_model
		));
		viewer.addMesh(fruit_mesh, {0.8, 0.8, 0.8}, 1.0);
	}

	VtkLineSegmentsVisualization sightlines(1, 0, 1);
	viewer.addActor(sightlines.getActor());

	// Create the fruit points visualization
	std::vector<VtkLineSegmentsVisualization> fruit_points_visualizations;
	for (const auto &scannable_points: all_scannable_points) {
		fruit_points_visualizations.push_back(createFruitLinesVisualization(scannable_points));
		viewer.addActor(fruit_points_visualizations.back().getActor());
	}


	std::vector<SeenPoints> ever_seen;
	ever_seen.reserve(all_scannable_points.size());
	for (const auto &scannable_points: all_scannable_points) {
		ever_seen.push_back(SeenPoints::create_all_unseen(scannable_points));
	}

	vtkNew<vtkTextActor> textActor;
	textActor->GetProperty()->SetColor(0.0, 0.0, 0.0);
	textActor->SetPosition(10, 0);
	textActor->SetInput("Orbiting the tree");
	viewer.addActor((vtkActor *) textActor.Get());

	// Register the timer callback function to be called at regular intervals
	viewer.addTimerCallback([&]() {

		// Advance the path point along the path
		if (advancePathPointClamp(path, path_point, interpolation_speed, equal_weights_max_distance)) {
			viewer.stop();
		}

		// Interpolate the robot's state
		auto interpolated_state = interpolate(path_point, path);

		// Update the robot's state in the visualization
		const auto fk = forwardKinematics(robot, interpolated_state.joint_values,
										  robot.findLinkByName("flying_base"), interpolated_state.base_tf);

		update_robot_state(robot, fk, robot_viz);

		// Get the position of the robot's end effector
		const auto &end_effector_position = fk.forLink(robot.findLinkByName("end_effector")).translation;

		std::vector<std::pair<math::Vec3d, math::Vec3d>> sightlines_data;

		// Update the visibility of the scannable points
		for (size_t fruit_i = 0; fruit_i < all_scannable_points.size(); ++fruit_i) {
			for (size_t i = 0; i < all_scannable_points[fruit_i].surface_points.size(); ++i) {
				if (!ever_seen[fruit_i].ever_seen[i] &&
					is_visible(all_scannable_points[fruit_i], i, end_effector_position)) {
					ever_seen[fruit_i].ever_seen[i] = true;

					// Add the sightline to the sightlines_data vector
					sightlines_data.push_back({end_effector_position,
											   all_scannable_points[fruit_i].surface_points[i].position});
				}
			}
			fruit_points_visualizations[fruit_i].setColors(generateVisualizationColors(ever_seen[fruit_i]));
		}

		size_t seen_total = 0;
		size_t unique_total = 0;

		for (size_t fruit_i = 0; fruit_i < all_scannable_points.size(); ++fruit_i) {
			seen_total += std::count(ever_seen[fruit_i].ever_seen.begin(), ever_seen[fruit_i].ever_seen.end(), true);
			unique_total += std::any_of(ever_seen[fruit_i].ever_seen.begin(),
										ever_seen[fruit_i].ever_seen.end(),
										[](bool b) { return b; });
		}

		// Calculate metrics
		double percent_surface_points_seen =
				100.0 * static_cast<double>(seen_total) / static_cast<double>(NUM_POINTS * all_scannable_points.size());
		// Round it:
		percent_surface_points_seen = std::round(percent_surface_points_seen * 100) / 100;

		std::stringstream ss;
		ss << "Percent of surface points seen: " << percent_surface_points_seen << "%" << std::endl;
		ss << "Number of fruits seen: " << unique_total << " out of " << all_scannable_points.size() << std::endl;

		textActor->SetInput(ss.str().c_str());

		// Update the sightlines visualization
		sightlines.updateLine(sightlines_data);

	});


	viewer.setCameraTransform({4.0, 4.0, 3.5}, {0.0, 0.0, 2.5});

	viewer.lockCameraUp();

	// Start the viewer
	viewer.start();
}

#define CREATE_SLIDER(widget, rep, minValue, maxValue, initValue, title, posY) \
    vtkNew<vtkSliderRepresentation2D> rep##_rep; \
    rep##_rep->SetMinimumValue(minValue); \
    rep##_rep->SetMaximumValue(maxValue); \
    rep##_rep->SetValue(initValue); \
    rep##_rep->SetTitleText(title); \
    rep##_rep->GetPoint1Coordinate()->SetCoordinateSystemToNormalizedDisplay(); \
    rep##_rep->GetPoint1Coordinate()->SetValue(0.2, posY); \
    rep##_rep->GetPoint2Coordinate()->SetCoordinateSystemToNormalizedDisplay(); \
    rep##_rep->GetPoint2Coordinate()->SetValue(0.8, posY); \
    vtkNew<vtkSliderWidget> widget; \
    widget->SetInteractor(viewer.renderWindowInteractor); \
    widget->SetRepresentation(rep##_rep); \
    widget->EnabledOn();

REGISTER_VISUALIZATION(max_distance) {

	// Load the tree meshes
	auto tree_model = tree_meshes::loadTreeMeshes("appletree");

	// Initialize a random number generator
	random_numbers::RandomNumberGenerator rng;

	// Create a robot model
	const auto &robot = experiments::createProceduralRobotModel();
	const robot_model::RobotModel::LinkId flying_base = robot.findLinkByName("flying_base");

	// Define the center of the tree
	const auto leaves_aabb = mesh_aabb(tree_model.leaves_mesh);
	math::Vec3d tree_center = leaves_aabb.center();
	double tree_radius = leaves_aabb.size().x() / 2.0;

	math::Vec3d approach_direction = {0, 1, 0};

	// Create a RobotPath that approaches the tree center.
	RobotPath path;
	path.states.push_back(fromEndEffectorAndVector(robot,
												   tree_center + approach_direction * tree_radius * 3.0,
												   approach_direction));
	path.states.push_back(fromEndEffectorAndVector(robot,
												   tree_center + approach_direction * tree_radius,
												   approach_direction));

	// Visualize the robot's path
	auto robot_viz = vizualisation::vizualize_robot_state(
			viewer,
			robot,
			forwardKinematics(
					robot, path.states[0].joint_values,
					flying_base,
					path.states[0].base_tf
			)
	);

	// Define the current position on the path
	PathPoint path_point = {0, 0.0};

	// Define the speed of interpolation
	double interpolation_speed = 0.02;

	// Add the tree trunk mesh to the viewer
	viewer.addMesh(tree_model.trunk_mesh, {0.5, 0.3, 0.1}, 1.0);

	// Define constants for the scannable points
	const size_t NUM_POINTS = 200;
	const double MAX_DISTANCE = 3.0;
	const double MIN_DISTANCE = 0;
	const double MAX_ANGLE = M_PI;

//	viewer.addMesh(tree_model.leaves_mesh, {0.1, 0.5, 0.1}, 1.0);
//	auto mesh_occlusion_model = std::make_shared<MeshOcclusionModel>(tree_model.leaves_mesh, 0.0);

	// Create the scannable points
	std::vector<ScannablePoints> all_scannable_points;
	for (const auto &fruit_mesh: tree_model.fruit_meshes) {
		all_scannable_points.push_back(createScannablePoints(
				rng,
				fruit_mesh,
				NUM_POINTS,
				MAX_DISTANCE,
				MIN_DISTANCE,
				MAX_ANGLE,
				std::nullopt
		));
		viewer.addMesh(fruit_mesh, {0.8, 0.8, 0.8}, 1.0);
	}
	vtkNew<vtkSphereSource> view_distance_source;
	view_distance_source->SetRadius(MAX_DISTANCE);

	vtkNew<vtkPolyDataMapper> view_distance_mapper;
	view_distance_mapper->SetInputConnection(view_distance_source->GetOutputPort());

	vtkNew<vtkActor> view_distance_actor;
	view_distance_actor->SetMapper(view_distance_mapper);
	view_distance_actor->GetProperty()->SetColor(0.0, 0.0, 1.0);
	view_distance_actor->GetProperty()->SetOpacity(0.1);
	viewer.addActor(view_distance_actor);

	CREATE_SLIDER(radius_slider, radius, 0.0, 3.0, MAX_DISTANCE, "Max distance", 0.1);
	CREATE_SLIDER(path_slider, path, 0.0, 1.0, 0.0, "Path", 0.2);

	VtkLineSegmentsVisualization sightlines(1, 0, 1);
	viewer.addActor(sightlines.getActor());

	// Create the fruit points visualization
	std::vector<VtkLineSegmentsVisualization> fruit_points_visualizations;
	for (const auto &scannable_points: all_scannable_points) {
		fruit_points_visualizations.push_back(createFruitLinesVisualization(scannable_points));
		viewer.addActor(fruit_points_visualizations.back().getActor());
	}

	// Register the timer callback function to be called at regular intervals
	viewer.addTimerCallback([&]() {

		// Get the max distance from the slider
		double max_distance = radius_rep->GetValue();
		view_distance_source->SetRadius(max_distance);
		for (auto &scannable_points: all_scannable_points) {
			scannable_points.max_distance = max_distance;
		}

//		advancePathPointWrap(path, path_point, interpolation_speed, equal_weights_max_distance);
		path_point.segment_i = 0;
		path_point.segment_t = path_rep->GetValue();

		// Interpolate the robot's state
		auto interpolated_state = interpolate(path_point, path);

		// Update the robot's state in the visualization
		const auto fk = forwardKinematics(robot, interpolated_state.joint_values,
										  robot.findLinkByName("flying_base"), interpolated_state.base_tf);

		update_robot_state(robot, fk, robot_viz);

		// Get the position of the robot's end effector
		const auto &end_effector_position = fk.forLink(robot.findLinkByName("end_effector")).translation;

		std::vector<std::pair<math::Vec3d, math::Vec3d>> sightlines_data;

		// Update the visibility of the scannable points
		for (auto &all_scannable_point: all_scannable_points) {
			for (size_t i = 0; i < all_scannable_point.surface_points.size(); ++i) {
				if (is_visible(all_scannable_point, i, end_effector_position)) {
					// Add the sightline to the sightlines_data vector
					sightlines_data.push_back({end_effector_position, all_scannable_point.surface_points[i].position});
				}
			}
		}

		// Update the sightlines visualization
		sightlines.updateLine(sightlines_data);

		// Set the view distance actor to the end-effector position
		view_distance_actor->SetPosition(end_effector_position[0], end_effector_position[1], end_effector_position[2]);

	});

	viewer.setCameraTransform({4.0, 4.0, 3.5}, {2.0, 0.0, 2.5});

	viewer.lockCameraUp();

	// Start the viewer
	viewer.start();
}