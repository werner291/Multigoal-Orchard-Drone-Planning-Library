// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/19/24.
//


#include <CGAL/convex_hull_3.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include "../planning/RobotModel.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../planning/fcl_utils.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../planning/state_tools.h"
#include "../planning/collision_detection.h"
#include "../planning/goal_sampling.h"
#include "../visualization/VtkTriangleSetVisualization.h"
#include "../visualization/robot_state.h"
#include "../planning/shell_path.h"
#include "../planning/probing_motions.h"
#include <vtkActor.h>
#include <vtkRenderer.h>

#include "../visualization/visualization_function_macros.h"
#include "../experiment_utils/default_colors.h"
#include "../visualization/RunQueue.h"
#include "../planning/swept_volume_ccd.h"

import approach_by_pullout;

using namespace mgodpl;
using namespace mgodpl::cgal;
using namespace mgodpl::approach_planning;

const math::Vec3d FLOOR_COLOR{0.3, 0.6, 0.3};

VtkTriangleSetVisualization convex_hull_viz(const Surface_mesh &convex_hull) {
	VtkTriangleSetVisualization viz(0.8, 0.8, 0.8, 0.5);

	std::vector<std::array<math::Vec3d, 3>> convex_hull_triangles;

	for (const auto &face: convex_hull.faces()) {
		auto vit = convex_hull.vertices_around_face(convex_hull.halfedge(face)).begin();

		Point_3 a = convex_hull.point(*vit++);
		Point_3 b = convex_hull.point(*vit++);
		Point_3 c = convex_hull.point(*vit++);

		convex_hull_triangles.push_back({
												math::Vec3d{a.x(), a.y(), a.z()},
												math::Vec3d{b.x(), b.y(), b.z()},
												math::Vec3d{c.x(), c.y(), c.z()}
										});
	}

	viz.updateTriangles(convex_hull_triangles);
	return viz;
}

std::vector<bool> visited_by_path(const std::vector<math::Vec3d> &targets, const RobotPath &path,
								  const robot_model::RobotModel &robot) {
	std::vector<bool> visited(targets.size(), false);

	auto end_effector = robot.findLinkByName("end_effector");

	for (const auto &state: path.states) {
		auto fk = robot_model::forwardKinematics(robot, state.joint_values, robot.findLinkByName("flying_base"),
												 state.base_tf);
		auto ee_pose = fk.forLink(end_effector).translation;

		for (size_t i = 0; i < targets.size(); ++i) {
			if ((targets[i] - ee_pose).norm() < 0.01) {
				visited[i] = true;
			}
		}
	}

	return visited;
}

REGISTER_VISUALIZATION(probing_fullpath) {
	const auto &robot = experiments::createProceduralRobotModel();

	const auto &tree_model = tree_meshes::loadTreeMeshes("appletree");

	auto start_time = std::chrono::high_resolution_clock::now();

	// Create a state outside the tree model.
	RobotState initial_state = fromEndEffectorAndVector(robot, {0, 5, 5}, {0, 1, 1});

	robot_model::RobotModel::LinkId flying_base = robot.findLinkByName("flying_base");

	const auto &tree_trunk_bvh = fcl_utils::meshToFclBVH(tree_model.trunk_mesh);
	fcl::CollisionObjectd tree_trunk_object(tree_trunk_bvh);

	CgalMeshData mesh_data(tree_model.leaves_mesh);

	// First, get some stats on how many straight-in motions we can do.
	const std::vector<math::Vec3d> &targets = computeFruitPositions(tree_model);

	ShellPathPlanningMethod planner;

	// Plan the final path as a whole:
	RobotPath final_path = planner.plan_static(robot,
											   tree_model.trunk_mesh,
											   tree_model.leaves_mesh,
											   targets,
											   initial_state);

	auto end_time = std::chrono::high_resolution_clock::now();

	std::cout << "Planning took " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).
			count() << " ms." << std::endl;

	// Check collision-freeness.
	{
		PathPoint collision_point;
		if (check_path_collides(robot, tree_trunk_object, final_path, collision_point)) {
			std::cout << "Final path collides at segment " << collision_point.segment_i << " at " << collision_point.
					segment_t << std::endl;
		} else {
			std::cout << "Final path is collision-free." << std::endl;
		}
	}

	// Run visualization of the final result.

	// Lock the camera's up direction to prevent it from rotating
	viewer.lockCameraUp();

	// Add the tree model's trunk mesh to the viewer with a wood color
	viewer.addMesh(tree_model.trunk_mesh, WOOD_COLOR);

	// Determine which targets have been visited by the path
	const auto &visited = visited_by_path(targets, final_path, robot);

	// For each target, add a sphere to the viewer at the target's position
	// The sphere's color is yellow if the target has been visited, and red otherwise
	for (size_t target_i = 0; target_i < targets.size(); ++target_i) {
		// Yellow if visited, red if not.
		math::Vec3d color = visited[target_i] ? math::Vec3d{1, 1, 0} : math::Vec3d{1, 0, 0};

		viewer.addSphere(0.05, targets[target_i], color, 1.0);
	}

	// Create a visualization of the convex hull and add it to the viewer
	VtkTriangleSetVisualization chull_viz = convex_hull_viz(mesh_data.convex_hull);
	viewer.addActor(chull_viz.getActor());

	// Visualize the initial state of the robot
	auto robot_viz = mgodpl::vizualisation::vizualize_robot_state(viewer, robot,
																  robot_model::forwardKinematics(
																		  robot, initial_state.joint_values,
																		  flying_base, initial_state.base_tf));

	// Initialize the segment time
	double segment_t = 0.0;

	// Add a timer callback to the viewer that will be called at regular intervals
	viewer.addTimerCallback([&]() {
		// Determine the current segment index and the fractional part of the segment time
		size_t segment_i = std::floor(segment_t);
		double segment_t_frac = segment_t - (double) segment_i;

		// If we have reached the end of the path, stop the viewer
		if (segment_i + 1 >= final_path.states.size()) {
			viewer.stop();
			return;
		} else {
			std::cout << "Segment " << segment_i << " / " << final_path.states.size() << std::endl;
		}

		// Interpolate between the current and next state based on the fractional part of the segment time
		const auto &state1 = final_path.states[segment_i];
		const auto &state2 = final_path.states[segment_i + 1];
		auto interpolated_state = interpolate(state1, state2, segment_t_frac);

		// Compute the forward kinematics for the interpolated state
		auto fk = robot_model::forwardKinematics(robot, interpolated_state.joint_values, flying_base,
												 interpolated_state.base_tf);

		// Check if the interpolated state collides with the tree trunk
		bool collides = check_robot_collision(robot, tree_trunk_object, interpolated_state);

		// Update the robot's state in the visualization
		update_robot_state(robot, fk, robot_viz);

		// Compute the length of the segment
		double segment_length = equal_weights_max_distance(state1, state2);

		// If the segment length is very small, set it to a minimum value to prevent division by zero
		if (segment_length < 0.1) {
			segment_length = 0.1;
		}

		// If the state collides, advance the segment time slowly, otherwise advance it faster
		if (collides) {
			segment_t += 0.01 / segment_length;
		} else {
			segment_t += 0.05 / segment_length;
		}
	});

	// Set the camera's position and target
	viewer.setCameraTransform({8, 0, 2}, {0, 0, 2});

	// Start the viewer
	viewer.start();
}

using namespace mgodpl;
using namespace visualization;
using namespace vizualisation; // Grr, need to check spelling.

REGISTER_VISUALIZATION(probing_isolated) {

	// In this visualization, we will show the process of finding a probing motion for target points in isolation.
	const auto &robot = experiments::createProceduralRobotModel();

	const auto &tree_model = tree_meshes::loadTreeMeshes("appletree");

	viewer.addMesh(tree_model.trunk_mesh, WOOD_COLOR);
	viewer.addMesh(tree_model.leaves_mesh, LEAF_COLOR);
	// Add the apple models:
	for (const auto &apple: tree_model.fruit_meshes) {
		viewer.addMesh(apple, FRUIT_COLOR);
	}

	auto start_time = std::chrono::high_resolution_clock::now();

	// Create a state outside the tree model.
	RobotState initial_state = fromEndEffectorAndVector(robot, {0, 5, 5}, {0, 1, 1});

	// Look up special links:
	robot_model::RobotModel::LinkId flying_base = robot.findLinkByName("flying_base");
	robot_model::RobotModel::LinkId end_effector = robot.findLinkByName("end_effector");

	const auto &tree_trunk_bvh = fcl_utils::meshToFclBVH(tree_model.trunk_mesh);
	fcl::CollisionObjectd tree_trunk_object(tree_trunk_bvh);

	CgalMeshData mesh_data(tree_model.leaves_mesh);

	const std::vector<math::Vec3d> &targets = computeFruitPositions(tree_model);

	size_t target_i = 0;

	size_t attempts_for_target = 0;
	const size_t MAX_ATTEMPTS = 100;

	const size_t MAX_TARGETS_FOR_VIDEO = 10;

	// Keep a step timer to slow down the visualization:
	size_t step_timer = 10;

	// RNG:
	random_numbers::RandomNumberGenerator rng(42);

	std::vector<vtkSmartPointer<vtkActor>> clear_actors;

	const auto tree_center = mesh_aabb(tree_model.leaves_mesh).center();

	viewer.addTimerCallback([&]() {

		auto rel_vec = (targets[target_i] - tree_center).normalized() * 10.0;
		rel_vec.z() = 5.0;

		// Focus the camera on the new target:
		viewer.setCameraTransform(
				targets[target_i] + rel_vec,
				targets[target_i]);

		if (step_timer > 0) {
			step_timer -= 1;
			return;
		} else {
			step_timer = 5;
		}

		// Clear actors:
		for (const auto &actor: clear_actors) {
			viewer.removeActor(actor);
		}
		clear_actors.clear();

		attempts_for_target += 1;
		if (attempts_for_target > MAX_ATTEMPTS) {
			attempts_for_target = 0;

			size_t max_targets = viewer.isRecording() ? MAX_TARGETS_FOR_VIDEO : targets.size();

			if (target_i + 1 >= max_targets) {
				if (viewer.isRecording()) {
					viewer.stop();
				} else {
					target_i = 0;
				}
				return;
			} else {
				target_i += 1;
			}
		}

		const math::Vec3d &target = targets[target_i];

		std::cout << "Target: " << target << "(" << target_i << "/" << targets.size() << ")" << std::endl;

		// Take 1000 samples:
		RobotState sample = genGoalStateUniform(rng, target, robot, flying_base, end_effector);

		std::cout << "Visualizing actors." << std::endl;
		auto fk = robot_model::forwardKinematics(robot, sample.joint_values, flying_base, sample.base_tf);

		// Check collision:
		bool collides = check_robot_collision(robot, tree_trunk_object, sample);

		auto actors = vizualize_robot_state(viewer,
											robot,
											fk,
											collides ? math::Vec3d{1, 0, 0} : math::Vec3d{0, 1, 0});

		for (const auto &actor: actors.actors) {
			clear_actors.push_back(actor);
		}

		if (!collides) {

			const auto &motion = straightout(robot, sample, mesh_data.tree, mesh_data.mesh_path);

			bool motion_collides = check_path_collides(robot, tree_trunk_object, motion.path);

			if (!motion_collides) {
				// Generate more actors:
				for (size_t j = 1; j < 10; ++j) {
					double t = j / 10.0;

					auto state = interpolate(PathPoint{0, t}, motion.path);

					auto fk = robot_model::forwardKinematics(robot, state.joint_values, flying_base, state.base_tf);

					auto actors = vizualize_robot_state(viewer, robot, fk, {0, 1, 0});

					for (const auto &actor: actors.actors) {
						clear_actors.push_back(actor);
					}
				}

				// Increase the step timer to slow down the visualization:
				step_timer = 50;

				// advance the target:
				attempts_for_target = MAX_ATTEMPTS;
			}
		}

	});

	viewer.lockCameraUp();

	viewer.start();

}

#include <optional>
#include <vector>

REGISTER_VISUALIZATION(static_goals_and_motions) {

	// In this visualization, we will show the process of finding a probing motion for target points in isolation.
	const auto &robot = experiments::createProceduralRobotModel();

	const auto &tree_model = tree_meshes::loadTreeMeshes("appletree");

	viewer.addMesh(tree_model.trunk_mesh, WOOD_COLOR);

	// Look up special links:
	robot_model::RobotModel::LinkId flying_base = robot.findLinkByName("flying_base");
	robot_model::RobotModel::LinkId end_effector = robot.findLinkByName("end_effector");

	const auto &tree_trunk_bvh = fcl_utils::meshToFclBVH(tree_model.trunk_mesh);
	fcl::CollisionObjectd tree_trunk_object(tree_trunk_bvh);

	CgalMeshData mesh_data(tree_model.leaves_mesh);

	const std::vector<math::Vec3d> &targets = computeFruitPositions(tree_model);

	random_numbers::RandomNumberGenerator rng(42);

	struct GoalAndMotion {
		std::optional<RobotState> goalConfiguration;
		std::optional<RobotPath> probingMotion;
	};

	std::vector<GoalAndMotion> goalAndMotions(targets.size());

	for (size_t i = 0; i < targets.size(); ++i) {
		const math::Vec3d &target = targets[i];

		auto pullout = plan_approach_by_pullout(tree_trunk_object, mesh_data, target, 0.0, robot, rng, 1000,
												{{
														 .sampled_state = [&](
																 const RobotState &state,
																 bool collision_free) {
															 if (collision_free) {
																 goalAndMotions[i].goalConfiguration = state;
															 }
														 },
														 .pullout_motion_considered = [&](
																 const ApproachPath &path,
																 bool path_collision_free) {
															 // Ignore it, we'll get it from the return value.
														 }
												 }});

		goalAndMotions[i].probingMotion = pullout ? std::make_optional(pullout->path) : std::nullopt;
	}

	// Lock the camera's up direction to prevent it from rotating
	viewer.lockCameraUp();

	// Let's visualize the results:
	for (size_t i = 0; i < targets.size(); ++i) {
		const math::Vec3d &target = targets[i];

		// Change the color of the fruit based on whether there's anything there at all:
		math::Vec3d color = goalAndMotions[i].goalConfiguration.has_value() ? math::Vec3d{1, 1, 0} : math::Vec3d{1, 0,
																												 0};

		// Add a sphere to the viewer at the target's position
		viewer.addMesh(tree_model.fruit_meshes[i], color);

		if (goalAndMotions[i].goalConfiguration.has_value()) {
			const auto &goal = goalAndMotions[i].goalConfiguration.value();

			auto fk = robot_model::forwardKinematics(robot, goal.joint_values, flying_base, goal.base_tf);

			// Visualize the goal state; change the color to highlight ones that don't have a probing motion:
			math::Vec3d robot_color = goalAndMotions[i].probingMotion.has_value() ? math::Vec3d{0.5, 0.5, 0.5}
																				  : math::Vec3d{1, 0, 0};
			auto robot_viz = mgodpl::vizualisation::vizualize_robot_state(viewer, robot, fk, robot_color);

			if (goalAndMotions[i].probingMotion.has_value()) {
				const auto &motion = goalAndMotions[i].probingMotion.value();

				// Visualize the probing motion
				auto fk = robot_model::forwardKinematics(robot, motion.end());

				auto robot_viz = mgodpl::vizualisation::vizualize_robot_state(viewer, robot, fk, {0, 1, 0});

				auto volume = swept_volume_triangles(robot, motion.states.front(), motion.states.back(), 1);

				// Visualize the swept volume
				VtkTriangleSetVisualization swept_volume_viz(0.8, 0.8, 0.8, 0.2);
				swept_volume_viz.updateTriangles(volume);
				viewer.addActor(swept_volume_viz.getActor());
			} else {
				std::cout << "Target " << i << " has no probing motion but does have a goal configuration."
						  << std::endl;
			}
		}
	}

	viewer.start();
}