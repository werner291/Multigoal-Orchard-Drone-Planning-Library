// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 8/28/24.
//

#include <vtkTextActor.h>
#include "../experiment_utils/default_colors.h"
#include "../experiment_utils/procedural_fruit_placement.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../planning/ParametricInfiniteCone.h"
#include "../planning/RobotPath.h"
#include "../planning/RobotPathFn.h"
#include "../planning/state_tools.h"
#include "../visualization/VtkLineSegmentVizualization.h"
#include "../visualization/VtkPolyLineVisualization.h"
#include "../visualization/robot_state.h"
#include "../visualization/visualization_function_macros.h"
#include "../visualization/vtk.h"
#include "../experiment_utils/surface_points.h"
#include "../visualization/scannable_points.h"

using namespace mgodpl;
using namespace mgodpl::vizualisation;

/**
 * Creates a path that orbits around the fruit, keeping the end-effector at a fixed distance from the fruit,
 * and the arm pointing radially inwards towards the fruit. This causes the robot's main body to swing out a lot.
 *
 * @param robot_model		The robot model that the path is for.
 * @param distance			The distance from the center of the fruit to the end-effector.
 * @param fruit_position	The position of the fruit.
 *
 * @return A function that takes a time parameter and returns a RobotState. (Note: captures robot_model, distance,
 * fruit_position by reference)
 */
RobotPathFn whole_body_orbit_path(const robot_model::RobotModel &robot_model,
								  double distance = 0.5,
								  const math::Vec3d &fruit_position = {0.0, 0.0, 0.0}) {
	return [robot_model, distance, fruit_position](double t) {
		const double angle = t * 2.0 * M_PI;
		const math::Vec3d arm_vector = math::Vec3d{cos(angle), sin(angle), 0.0};
		const math::Vec3d offset_vector = fruit_position + arm_vector * distance;
		return fromEndEffectorAndVector(robot_model, offset_vector, arm_vector);
	};
}

RobotPathFn end_effector_orbit_path(const robot_model::RobotModel &robot_model, double distance = 0.5) {
	return [robot_model, distance](double t) {
		const double angle = t * 2.0 * M_PI;
		const math::Vec3d arm_vector = math::Vec3d{1.0, 0.0, 0.0};
		const math::Vec3d offset_vector = math::Vec3d{cos(angle), sin(angle), 0.0} * distance;
		return fromEndEffectorAndVector(robot_model, offset_vector, arm_vector);
	};
}

RobotPathFn end_effector_vertical_path(const robot_model::RobotModel &robot_model, double distance = 0.5) {
	return [robot_model, distance](double t) {
		const double angle = t * 2.0 * M_PI;
		const math::Vec3d arm_vector = math::Vec3d{1.0, 0.0, 0.0};
		const math::Vec3d offset_vector = math::Vec3d{0.0, cos(angle), sin(angle)} * distance;
		return fromEndEffectorAndVector(robot_model, offset_vector, arm_vector);
	};
}

/**
 * A composition of slices of the above two:
 *
 * We follow a
 */
RobotPathFn segmented_orbit_path(const robot_model::RobotModel &robot_model, double distance = 0.5) {
	return concat({whole_body_orbit_path(robot_model, distance), end_effector_orbit_path(robot_model, distance)});
}

REGISTER_VISUALIZATION(parametric_scan_path) {

	// The radius of the sphere representing the fruit.
	const double FRUIT_RADIUS = 0.1;

	// The distance from the surface at which we'd ideally like to scan the fruit.
	const double SCAN_DISTANCE = 0.2;

	double scan_radius = FRUIT_RADIUS + SCAN_DISTANCE;

	const math::Vec3d fruit_position = {0.0, 0.0, 0.0};
	viewer.addSphere(FRUIT_RADIUS, fruit_position, FRUIT_COLOR);

	const robot_model::RobotModel robot_model = experiments::createProceduralRobotModel(
			experiments::RobotArmParameters{.total_arm_length = 1.0, .joint_types = {experiments::HORIZONTAL}});

	const math::Vec3d initial_end_effector_position = {scan_radius, 0.0, 0.0};
	const math::Vec3d initial_arm_vec(1.0, 0.0, 0.0);

	const RobotState initial_state = fromEndEffectorAndVector(robot_model, {scan_radius, 0.0, 0.0}, initial_arm_vec);

	viewer.setCameraTransform(fruit_position + math::Vec3d{5.0, 0.0, 5}, fruit_position);

	auto rb = vizualize_robot_state(viewer, robot_model, forwardKinematics(robot_model, initial_state));

	std::vector<math::Vec3d> cone_points;

	ParametricInfiniteCone cone(-initial_arm_vec, M_PI / 4.0, initial_end_effector_position);

	// Generate points on the cone by iterating through t and theta.
	for (int t_i = 0; t_i < 10; t_i++) {
		const double t = t_i / 10.0;
		for (int theta_i = 0; theta_i < 10; theta_i++) {
			const double theta = theta_i / 10.0 * 2.0 * M_PI;
			cone_points.push_back(cone.position(t, theta));
		}
	}

	VtkPointCloudVisualization points(1, 0, 1);
	points.updatePoints(cone_points);
	viewer.addActor(points.getActor());

	// Something fun: let's make a circle, but through t and theta space:
	std::vector<math::Vec3d> circle_points;
	VtkPolyLineVisualization circle(1, 1, 1);
	viewer.addActor(circle.getActor());

	int path_i = 0;

	viewer.addTimerCallback([&]() {
		int i = circle_points.size();
		double t = static_cast<double>(i) / 100.0;

		switch (path_i) {
			case 0: {
				// Weird curve path
				const double theta = 2.0 * M_PI * t;
				circle_points.push_back(cone.position(sin(theta / 2.0) * scan_radius * 1.5, theta));
			} break;
			case 1: {
				// Spider path
				double theta = std::floor(t * 8.0) * 2.0 * M_PI / 8.0;
				double linear = t * 8.0 - std::floor(t * 8.0);
				if (linear > 0.5)
					linear = 1.0 - linear;
				circle_points.push_back(cone.position(linear, theta));
			} break;
			default:
				throw std::runtime_error("Invalid path_i");
		}

		circle.updateLine(circle_points);

		if (i == 100) {
			circle_points.clear();
			path_i = (path_i + 1) % 2;
		}
	});


	viewer.start();
}

math::Vec3d arm_vector_from_eepoint(const math::Vec3d &ee_point) {
	const math::Vec3d REFERENCE_SHALLOW_PULL_POINT = {5, 0, 0};
	const math::Vec3d arm_vector = (REFERENCE_SHALLOW_PULL_POINT - ee_point).normalized();
	return arm_vector;
}

RobotPathFn weird_cone_curve_path(const robot_model::RobotModel &robot_model, double distance = 0.5) {
	ParametricInfiniteCone cone({-1.0, 0.0, 0.0}, M_PI / 4.0, {distance, 0.0, 0.0});

	return [robot_model, cone, distance](double t) {
		const double theta = t * 2.0 * M_PI;
		const auto ee_point = cone.position(sin(theta / 2.0) * distance * 1.2, theta);
		return fromEndEffectorAndVector(robot_model, ee_point, arm_vector_from_eepoint(ee_point));
	};
}

RobotPathFn spider_path(const robot_model::RobotModel &robot_model, double distance = 0.5) {
	ParametricInfiniteCone cone({-1.0, 0.0, 0.0}, M_PI / 4.0, {distance, 0.0, 0.0});

	return [robot_model, cone, distance](double t) {
		const double theta = std::floor(t * 8.0) * 2.0 * M_PI / 8.0;
		double linear = t * 8.0 - std::floor(t * 8.0);
		if (linear > 0.5)
			linear = 1.0 - linear;
		const auto ee_point = cone.position(linear, theta);
		return fromEndEffectorAndVector(robot_model, ee_point, arm_vector_from_eepoint(ee_point));
	};
}

RobotPathFn curved_spider_path(const robot_model::RobotModel &robot_model, double distance = 0.5) {

	return [robot_model, distance](double t) {
		const double theta = std::floor(t * 8.0) * 2.0 * M_PI / 8.0;
		double linear = t * 8.0 - std::floor(t * 8.0);
		if (linear > 0.5)
			linear = 1.0 - linear;

		math::Vec3d ee_point = {
				distance - linear * linear, sin(theta) * linear * distance * 4.0, cos(theta) * linear * distance * 4.0};

		return fromEndEffectorAndVector(robot_model, ee_point, arm_vector_from_eepoint(ee_point));
	};
}

ScannablePoints generate_sphere_scannable_points(const int n_points, random_numbers::RandomNumberGenerator &rng,
												  const math::Vec3d &fruit_position, const double FRUIT_RADIUS) {
	std::vector<SurfacePoint> points;
	// Generate points on the sphere by sampling from a normal distribution 3 times and normalizing:
	for (int i = 0; i < 1000; i++) {
		math::Vec3d point(rng.gaussian01(), rng.gaussian01(), rng.gaussian01());
		point.normalize();
		points.push_back({ .position = fruit_position+point * FRUIT_RADIUS, .normal = point});
	}

	// Create the scannable points
	ScannablePoints scannable_points {
			0.5, 0.1, M_PI/2.0, std::move(points)
	};

	return scannable_points;
}

#include <vtkRenderer.h>
#include <vtkActor2D.h>
#include <vtkTextProperty.h>

/**
 * Visualizes a set of different motions whereby the robot, starting from a configuration near a fruit,
 * will make motions to scan the fruit from different angles using its end-effector.
 */
REGISTER_VISUALIZATION(scanning_motions_straight_arm) {

	// Create a small vtk text label saying "Hello world!":
	vtkNew<vtkTextActor> stats_output;
	stats_output->SetInput("Scanning motions");
	stats_output->SetPosition(10, 10);
	stats_output->GetTextProperty()->SetFontSize(24);
	viewer.viewerRenderer->AddActor2D(stats_output);

	// The radius of the sphere representing the fruit.
	const double FRUIT_RADIUS = 0.1;

	// The distance from the surface at which we'd ideally like to scan the fruit.
	const double SCAN_DISTANCE = 0.1;

	double scan_radius = FRUIT_RADIUS + SCAN_DISTANCE;

	const math::Vec3d fruit_position = {0.0, 0.0, 0.0};
	viewer.addSphere(FRUIT_RADIUS, fruit_position, FRUIT_COLOR);

	// Generate points on the sphere surface:
	random_numbers::RandomNumberGenerator rng;

	const auto& scannable_points = generate_sphere_scannable_points(100, rng, fruit_position, FRUIT_RADIUS);

	// Initialize all points as unseen
	SeenPoints ever_seen = SeenPoints::create_all_unseen(scannable_points);

	// Create the fruit points visualization
	VtkLineSegmentsVisualization fruit_points_visualization = createFruitLinesVisualization(scannable_points);
	viewer.addActor(fruit_points_visualization.getActor());

	const robot_model::RobotModel robot_model = experiments::createProceduralRobotModel(
			experiments::RobotArmParameters{.total_arm_length = 1.0, .joint_types = {experiments::HORIZONTAL}, .add_spherical_wrist=false});

	const RobotState initial_state = fromEndEffectorAndVector(robot_model, {scan_radius, 0.0, 0.0}, {1.0, 0.0, 0.0});

	viewer.setCameraTransform(fruit_position + math::Vec3d{1.0, 4.0, 3}, fruit_position);

	auto rb = vizualize_robot_state(viewer, robot_model, forwardKinematics(robot_model, initial_state));

	const std::vector<std::tuple<RobotPathFn, double, std::string>> paths = {
		{whole_body_orbit_path(robot_model, scan_radius), 0.05, "whole_body_orbit_path"},
		{end_effector_orbit_path(robot_model, scan_radius), 0.05, "end_effector_orbit_path"},
		{end_effector_vertical_path(robot_model, scan_radius), 0.025, "end_effector_vertical_path"},
		{weird_cone_curve_path(robot_model, scan_radius), 0.025, "weird_cone_curve_path"},
		{spider_path(robot_model, scan_radius), 0.01, "spider_path"},
		{curved_spider_path(robot_model, scan_radius), 0.01, "curved_spider_path"}
	};

	double t = 0.0;

	VtkLineSegmentsVisualization to_surface(1, 0, 1);
	viewer.addActor(to_surface.getActor());

	VtkPolyLineVisualization ee_trace_vis(1, 1, 0);
	viewer.addActor(ee_trace_vis.getActor());

	std::vector<std::pair<math::Vec3d, math::Vec3d>> lines;
	std::vector<math::Vec3d> ee_trace;

	double robot_distance = 0.0;
	size_t current_path = 0;
	RobotState last_state = get<0>(paths[current_path])(0.0);
	stats_output->SetInput(get<2>(paths[current_path]).c_str());

	viewer.addTimerCallback([&]() {

		t += get<1>(paths[current_path]);

		RobotState new_state = get<0>(paths[current_path])(t);

		// Add the distance:
		robot_distance += equal_weights_distance(last_state, new_state);
		last_state = new_state;

		if (t > 1.0) {

			std::cout << "Path " << get<2>(paths[current_path]) << " scan stats: " << ever_seen.count_seen() << "/" << scannable_points.surface_points.size() << ", distance: " << robot_distance << std::endl;

			t = 0.0;
			current_path += 1;
			if (current_path >= paths.size()) {
				current_path = 0;
				if (viewer.isRecording()) {
					viewer.stop();
				}
			}

			lines.clear();
			ee_trace.clear();
			last_state = get<0>(paths[current_path])(t);
			ever_seen = SeenPoints::create_all_unseen(scannable_points);
			robot_distance = 0.0;
			stats_output->SetInput(get<2>(paths[current_path]).c_str());
		}

		auto fk = forwardKinematics(robot_model, new_state);
		update_robot_state(robot_model, fk, rb);

		const math::Vec3d ee_pos = fk.forLink(robot_model.findLinkByName("end_effector")).translation;

		// Add the ee-to-fruit line to the visualization.
		lines.emplace_back(ee_pos, fruit_position);
		to_surface.updateLine(lines);

		// Add the end-effector trace to the visualization.
		ee_trace.push_back(ee_pos);
		ee_trace_vis.updateLine(ee_trace);

		// Update seen/unseen points
		update_visibility(scannable_points, ee_pos, ever_seen);

		// Set the colors of the fruit points visualization
		fruit_points_visualization.setColors(generateVisualizationColors(ever_seen));
	});

	viewer.start();
}
