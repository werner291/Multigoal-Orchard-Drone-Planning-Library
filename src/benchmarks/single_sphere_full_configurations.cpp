// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#include "../planning/RobotPath.h"

#include "benchmark_function_macros.h"
#include "../visualization/visualization_function_macros.h"
#include "../planning/RobotModel.h"
#include "../planning/RandomNumberGenerator.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../experiment_utils/scan_paths.h"
#include "../visualization/scannable_points.h"
#include "../visualization/robot_state.h"
#include "../visualization/scannable_points.h"
#include "../experiment_utils/surface_points.h"
#include "../experiment_utils/SensorModel.h"
#include "../planning/state_tools.h"
#include "../visualization/VtkPolyLineVisualization.h"

import visualization.ThrottledRunQueue;

using namespace mgodpl;

constexpr double TARGET_RADIUS = 0.1;

REGISTER_VISUALIZATION(single_start_straight_arm_single_target) {
	// Create a random number generator.
	random_numbers::RandomNumberGenerator rng(42);

	robot_model::RobotModel robot = experiments::createProceduralRobotModel({
		.total_arm_length = 1.0,
		.joint_types = {
			experiments::JointType::HORIZONTAL,
		},
		.add_spherical_wrist = false
	});

	RobotState initial_state = fromEndEffectorAndVector(robot,
	                                                    {1.0, 0.0, 0.0},
	                                                    {1.0, 0.0, 0.0});

	auto robot_actors = visualization::vizualize_robot_state(viewer, robot, initial_state);

	viewer.setCameraTransform({0.0, 8.0, 4.0}, {0.0, 0.0, 0.0});

	// Keep it simple: infinite distance, 180 degree field of view, 90 degree surface angle
	const experiments::SensorModelParameters sensor_model{
		.min_distance = 0.0,
		.max_distance = INFINITY,
		.surface_max_angle = M_PI / 2.0,
		.fov_angle = M_PI
	};

	// Number of points to sample
	size_t num_points = 1024;

	// Sample points on the sphere
	const auto points = sample_points_on_sphere_quasi_random(rng, num_points, TARGET_RADIUS);

	// Initialize all points as unseen
	SeenPoints ever_seen = SeenPoints::create_all_unseen(points);

	// Create the sphere points visualization
	VtkLineSegmentsVisualization sphere_points_visualization = createFruitLinesVisualization(points);

	// Add the sphere points visualization to the viewer
	viewer.addActor(sphere_points_visualization.getActor());

	// Add a sphere with the specified radius at the origin
	viewer.addSphere(TARGET_RADIUS, {0.0, 0.0, 0.0}, {0.8, 0.8, 0.8}, 1.0, 32);

	RobotPath path = RobotPath::singleton(initial_state);

	/// Generate a path; we'll start with a fixed equatorial orbit:
	constexpr size_t N_STEPS = 128;
	constexpr double radius = 1.0;

	for (size_t step = 0; step < N_STEPS; ++step) {
		const double t = static_cast<double>(step) / static_cast<double>(N_STEPS - 1);

		math::Vec3d ray = {
			std::cos(2.0 * M_PI * t), radius * std::sin(2.0 * M_PI * t), 0.0
		};

		math::Vec3d ee_pos = ray * radius;

		RobotState state = fromEndEffectorAndVector(robot, ee_pos, ray);
		path.append(state);
	}

	viewer.run_puppeteer_thread([&](visualization::ThrottledRunQueue &queue) {
		PathPoint path_point = {0, 0.0};

		const auto end_effector_link = robot.findLinkByName("end_effector");

		std::optional<RobotState> last_state;
		std::vector<std::pair<double, size_t> > distance_and_points_seen;

		do {
			RobotState interpolated_state = interpolate(path_point, path);
			auto fk = forwardKinematics(robot, interpolated_state.joint_values, 0, interpolated_state.base_tf);

			math::Vec3d ee_pos = fk.forLink(end_effector_link).translation;
			math::Vec3d ray = fk.forLink(end_effector_link).orientation.rotate({0, 1, 0});

			size_t points_seen = 0;
			for (size_t pt_i = 0; pt_i < points.size(); ++pt_i) {
				if (!ever_seen.ever_seen[pt_i] && sensor_model.is_visible(ee_pos, ray, points[pt_i])) {
					ever_seen.ever_seen[pt_i] = true;
					points_seen++;
				}
			}

			if (last_state) {
				double distance = (interpolated_state.base_tf.translation - last_state->base_tf.translation).norm();
				distance_and_points_seen.emplace_back(distance, points_seen);
			}

			last_state = interpolated_state;

			queue.run_main_void([&](const auto &_viewer) {
				update_robot_state(robot, fk, robot_actors);
				sphere_points_visualization.setColors(generateVisualizationColors(ever_seen));
			});
		} while (!advancePathPointClamp(path, path_point, 0.05, equal_weights_distance));
	});
}

/**
 * \struct ViewPoint
 * \brief Represents a viewpoint with a direction in 3D space.
 */
struct ViewPoint {
	math::Vec3d position; ///< The position of the viewpoint in 3D space.
	math::Vec3d direction; ///< The direction the viewpoint is facing.
};

/**
 * \brief Function type for embedding viewpoints into the robot state space.
 */
using ScanEmbedFn = std::function<RobotState(const ViewPoint &)>;

/**
 * A general-purpose visualization function that scans through a few viewpoints
 * and shows how that translates to the robot's state space.
 *
 * \param	viewer	The viewer to use for visualization.
 * \param	embed	The embedding function to use.
 * \param	robot	The robot model to use; must correspond to the robot model used in the embedding function.
 */
void visualize_embed_fn(SimpleVtkViewer &viewer, const ScanEmbedFn &embed, const robot_model::RobotModel &robot) {
	const math::Vec3d target_position = {0.0, 0.0, 0.0};

	// Create a grey sphere with radius TARGET_RADIUS at the origin.
	viewer.addSphere(TARGET_RADIUS, target_position, {0.8, 0.8, 0.8}, 1.0, 32);

	const RobotState initial_state = embed({target_position, {-1.0, 0.0, 0.0}});
	auto robot_actors = visualization::vizualize_robot_state(viewer, robot, initial_state);

	VtkPolyLineVisualization eye_positions_visualization(1, 0, 0);
	viewer.addActor(eye_positions_visualization.getActor());

	constexpr double SCAN_DISTANCE = TARGET_RADIUS * 3.0;

	const std::array<ParametricPath, 3> PATHS = {
		// Equatorial orbit:
		[&](const double &t) {
			const double angle = 2.0 * M_PI * t;
			return math::Vec3d(std::cos(angle), std::sin(angle), 0.0) * SCAN_DISTANCE;
		},
		// Tracing a latitude line:
		[&](const double &t) {
			const double angle = M_PI * (t - 0.5);
			return math::Vec3d(std::cos(angle), 0.0, std::sin(angle)) * SCAN_DISTANCE;
		},
		// Tracing a longitude line at latitude 45 degrees:
		[&](const double &t) {
			const double angle = 2.0 * M_PI * t;
			constexpr double latitude = M_PI / 4.0;
			return math::Vec3d(std::cos(angle) * std::cos(latitude),
			                   std::sin(angle) * std::cos(latitude),
			                   std::sin(latitude)) * SCAN_DISTANCE;
		}
	};

	viewer.run_puppeteer_thread([&](visualization::ThrottledRunQueue &queue) {
		viewer.setCameraTransform({0.0, 8.0, 4.0}, {0.0, 0.0, 0.0});

		for (const ParametricPath &path: PATHS) {
			// First, visualise a trace of the path we'll follow:
			std::vector<math::Vec3d> path_points;
			for (size_t i = 0; i < 100; ++i) {
				const double t = static_cast<double>(i) / 100.0;
				path_points.push_back(path(t) + target_position);
			}
			queue.run_main_void([&](const auto &_viewer) {
				eye_positions_visualization.updateLine(path_points);
			});

			const size_t num_points = 200;

			// Then, animate the robot along the path:
			for (size_t i = 0; i < num_points; ++i) {
				const double t = static_cast<double>(i) / static_cast<double>(num_points - 1);

				math::Vec3d position = path(t);
				math::Vec3d view_direction = target_position - position;

				queue.run_main_void([&](const auto &_viewer) {
					update_robot_state(robot, embed({position, view_direction}), robot_actors);
				});
			}
		}
	});
}

REGISTER_VISUALIZATION(embed_facing_straightarm) {
	// Create a random number generator.
	random_numbers::RandomNumberGenerator rng(42);

	robot_model::RobotModel robot = experiments::createProceduralRobotModel({
		.total_arm_length = 1.0,
		.joint_types = {
			experiments::JointType::HORIZONTAL,
		},
		.add_spherical_wrist = false
	});

	const ScanEmbedFn embed = [&](const ViewPoint &vp) {
		return fromEndEffectorAndVector(robot, vp.position, -vp.direction);
	};

	visualize_embed_fn(viewer, embed, robot);
}
