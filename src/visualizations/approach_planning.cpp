// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 9/12/24.
//

#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/default_colors.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../planning/RandomNumberGenerator.h"
#include "../planning/RobotState.h"
#include "../planning/collision_detection.h"
#include "../planning/fcl_utils.h"
#include "../planning/goal_sampling.h"
#include "../visualization/robot_state.h"
#include "../visualization/visualization_function_macros.h"
#include "../visualization/VtkLineSegmentVizualization.h"

#include "../planning/state_tools.h"
#include "../planning/cgal_chull_shortest_paths.h"
#include <optional>
#include <vector>
#include <vtkActor.h>

using namespace mgodpl;

import approach_makeshift_prm;

REGISTER_VISUALIZATION(nearby_sampling) {

	// In this visualization, we will show the process of sampling nearby states and states along a motion.
	const auto &robot = experiments::createProceduralRobotModel({
																		.total_arm_length = 1.0,
																		.joint_types = {
																				experiments::JointType::HORIZONTAL},
																});

	RobotState start_state{
			.base_tf = math::Transformd::identity(),
			.joint_values = {0.0}
	};

	auto initial_fk = robot_model::forwardKinematics(robot, start_state);

	mgodpl::vizualisation::vizualize_robot_state(viewer, robot, initial_fk, {1, 0, 1});

	random_numbers::RandomNumberGenerator rng(42);

	const double FACTOR = 0.5;

	int samples = 0;

	std::vector<vizualisation::RobotActors> last_actors;
	const size_t KEEP = 10;

	viewer.addTimerCallback([&]() {

		RobotState nearby_state = sample_nearby_state(start_state, rng, FACTOR);

		auto fk = robot_model::forwardKinematics(robot, nearby_state);

		while (last_actors.size() >= KEEP) {
			for (auto &actor: last_actors.front().actors) {
				viewer.removeActor(actor);
			}
			last_actors.erase(last_actors.begin());
		}

		// Draw the nearby state:
		last_actors.push_back(mgodpl::vizualisation::vizualize_robot_state(viewer, robot, fk, {0, 1, 0}));

	});

	viewer.lockCameraUp();

	viewer.setCameraTransform({5, 5, 1}, {0, 0, 0});

	viewer.start();
}


REGISTER_VISUALIZATION(makeshift_exponential_sampling) {

	// In this visualization, we will show the process of sampling nearby states and states along a motion.
	const auto &robot = experiments::createProceduralRobotModel({
																		.total_arm_length = 1.0,
																		.joint_types = {
																				experiments::JointType::HORIZONTAL},
																});

	RobotState start_state{
			.base_tf = math::Transformd::identity(),
			.joint_values = {0.0}
	};

	RobotState end_state{
			.base_tf = {
					.translation = {1, 1, 1},
					.orientation = math::Quaterniond::fromAxisAngle(math::Vec3d::UnitZ(), 1.0)
			},
			.joint_values = {M_PI_2}
	};

	auto initial_fk = robot_model::forwardKinematics(robot, start_state);

	for (int i = 0; i <= 10; ++i) {
		double t = i / 10.0;
		// Interpolate between the start and end state:
		RobotState interpolated_state = interpolate(start_state, end_state, t);
		auto fk = robot_model::forwardKinematics(robot, interpolated_state);
		mgodpl::vizualisation::vizualize_robot_state(viewer, robot, fk, {1, 0, 1});
	}

	random_numbers::RandomNumberGenerator rng(42);

	const double FACTOR = 2.0;

	int samples = 0;

	std::vector<vizualisation::RobotActors> last_actors;
	const size_t KEEP = 10;

	viewer.addTimerCallback([&]() {

		RobotState nearby_state = makeshift_exponential_sample_along_motion(start_state, end_state, rng, FACTOR);

		auto fk = robot_model::forwardKinematics(robot, nearby_state);

		while (last_actors.size() >= KEEP) {
			for (auto &actor: last_actors.front().actors) {
				viewer.removeActor(actor);
			}
			last_actors.erase(last_actors.begin());
		}

		// Draw the nearby state:
		last_actors.push_back(mgodpl::vizualisation::vizualize_robot_state(viewer, robot, fk, {0, 1, 0}));

	});

	viewer.lockCameraUp();

	viewer.setCameraTransform({5, 5, 1}, {0, 0, 0});

	viewer.start();
}

import rrt;
#include <CGAL/Side_of_triangle_mesh.h>

REGISTER_VISUALIZATION(difficult_apples_rrt) {
	const std::vector<size_t> DIFFICULT_APPLES = {7, 10, 41, 88};

	const auto &robot = experiments::createProceduralRobotModel(
			{
					.total_arm_length = 1.0,
					.joint_types = {experiments::JointType::HORIZONTAL},
			});

	const auto tree_mesh = mgodpl::tree_meshes::loadTreeMeshes("appletree");

	const auto targets = computeFruitPositions(tree_mesh);

	viewer.addMesh(tree_mesh.trunk_mesh, WOOD_COLOR);

	for (size_t apple: DIFFICULT_APPLES) {
		viewer.addMesh(tree_mesh.fruit_meshes[apple], FRUIT_COLOR);
	}

	const auto &tree_trunk_bvh = fcl_utils::meshToFclBVH(tree_mesh.trunk_mesh);
	fcl::CollisionObjectd tree_trunk_object(tree_trunk_bvh);

	random_numbers::RandomNumberGenerator rng(42);

	const auto base_link = robot.findLinkByName("flying_base");
	const auto end_effector_link = robot.findLinkByName("end_effector");

	cgal::CgalMeshData mesh_data(tree_mesh.leaves_mesh);

	// For each, find a goal state:
	for (size_t apple: DIFFICULT_APPLES) {
		std::cout << "Finding a path for apple " << apple << "..." << std::endl;
		for (int sample_i = 0; sample_i < 1000; ++sample_i) {
			auto sample = mgodpl::genGoalStateUniform(
					rng,
					targets[apple],
					0.0,
					robot,
					base_link,
					end_effector_link
			);

			if (check_robot_collision(robot, tree_trunk_object, sample)) {
				continue;
			}

			auto fk = robot_model::forwardKinematics(robot, sample);

			// Create a segment visualization:
			std::vector<std::pair<math::Vec3d, math::Vec3d>> segments;
			VtkLineSegmentsVisualization segment_viz(0, 1, 0);
			viewer.addActor(segment_viz.getActor());

			CGAL::Side_of_triangle_mesh<cgal::Surface_mesh, cgal::K> inside(mesh_data.convex_hull);

			std::optional<mgodpl::RobotPath> path;

			rrt(
					sample,
					[&]() {
						// Generate a state randomly:
						return generateUniformRandomState(robot, rng, 5, 10, M_PI_2);
					},
					[&](const RobotState &state) {
						// Check if the state is in collision:
						return check_robot_collision(robot, tree_trunk_object, state);
					},
					[&](const RobotState &from, const RobotState &to) {
						// Check if the motion is in collision:
						return check_motion_collides(robot, tree_trunk_object, from, to);
					},
					equal_weights_distance,
					1000,
					[&](const std::vector<RRTNode> &nodes) {

						math::Vec3d last_position = nodes.back().state.base_tf.translation;
						math::Vec3d last_position_2 = nodes[nodes.back().parent_index].state.base_tf.translation;

						segments.push_back({last_position_2, last_position});
						segment_viz.updateLine(segments);

						auto side = inside(cgal::to_cgal_point(last_position));

						if (side == CGAL::ON_UNBOUNDED_SIDE) {

							std::cout << "Found a way out for apple " << apple << "!" << std::endl;

							// Retrace:
							std::vector<RobotState> path_states;
							size_t current_index = nodes.size() - 1;

							do {
								path_states.push_back(nodes[current_index].state);
								current_index = nodes[current_index].parent_index;
							} while (current_index != 0);

							path_states.push_back(nodes[0].state);

							path = RobotPath{
									.states = path_states
							};

							return true;
						} else {
							return false;
						}
					}
			);

			if (path) {
				std::cout << "Found a path for apple " << apple << "!" << std::endl;

				// Visualize the path:
				PathPoint pt{0, 0.0};

				do {
					RobotState st = interpolate(pt, *path);
					auto fk = robot_model::forwardKinematics(robot, st);
					mgodpl::vizualisation::vizualize_robot_state(viewer, robot, fk, {1, 0, 1});
				} while (!advancePathPointClamp(*path, pt, 0.1, equal_weights_distance));

				break;
			}


		}
	}

	viewer.lockCameraUp();

	viewer.start();
}