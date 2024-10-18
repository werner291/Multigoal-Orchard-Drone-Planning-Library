// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#include "../experiment_utils/procedural_robot_models.h"
#include "../experiment_utils/tree_benchmark_data.h"
#include "../planning/RandomNumberGenerator.h"
#include "../planning/RobotModel.h"
#include "../planning/RobotPath.h"
#include "../visualization/RunQueue.h"
#include "../visualization/Throttle.h"
#include "../visualization/robot_state.h"
#include "../visualization/visualization_function_macros.h"
#include "../visualization/declarative.h"

#include <functional>
#include <memory>
#include <thread>
#include <vector>
#include <vtkActor.h>

#include "../planning/state_tools.h"

using namespace mgodpl;
using namespace visualization;
using namespace vizualisation;

import sampling;
import goal_sampling;
import collision_detection;
import collision_visualization;
import shell_state_projection;
import approach_makeshift_prm;
import rrt;
import visualization.ThrottledRunQueue;

using namespace mgodpl;
using namespace visualization;


REGISTER_VISUALIZATION(rrt_for_approach_planning) {
	auto robot = experiments::createProceduralRobotModel(
		{
			.total_arm_length = 1.0,
			.joint_types = {
				experiments::JointType::HORIZONTAL
			},
			.add_spherical_wrist = false
		});

	auto tree = experiments::loadBenchmarkTreemodelData("appletree");

	Throttle throttle;

	auto base_collision_fn =
			collision_check_fn_in_environment(
				{robot, *tree.tree_collision_object});

	auto run_queue = std::make_shared<RunQueue>();
	auto actors = std::make_shared<std::vector<RobotActors> >();

	auto motion_check_visualization_fn =
			create_motion_check_visualization_fn(
				base_collision_fn,
				actors,
				run_queue,
				throttle,
				robot);

	random_numbers::RandomNumberGenerator rng;

	auto sample = make_uniform_sampler_fn(robot, rng, tree.tree_mesh.leaves_mesh, 2.0);

	viewer.addTree(tree.tree_mesh, true, true);

	std::function sample_goal_t = goal_region_sampler(robot, rng);


	std::function check_goal_state = visualize_and_cleanup_state(base_collision_fn, run_queue, throttle, robot);

	auto accept_at = accept_outside_tree(*tree.tree_convex_hull);

	// We're going to be running the algorithm in a separate thread to keep the logic as clean as possible:
	std::thread algorithm_thread([&]() {
		for (const auto &target: tree.target_points) {
			std::function sample_goal = [&]() {
				return sample_goal_t(target);
			};

			// Try the RRT operation at valid goal samples.
			auto path = try_at_valid_goal_samples<RobotPath>(
				check_goal_state,
				sample_goal,
				1000,
				[&](const RobotState &goal) -> std::optional<RobotPath> {
					auto shell_state = project_to_shell_state(goal, *tree.tree_convex_hull, robot);

					auto biased_sampler = make_biased_sample_fn(goal,
					                                            shell_state,
					                                            rng,
					                                            1.0);

					// Run the RRT algorithm and try to find a path.
					return rrt_path_to_acceptable(
						goal,
						biased_sampler,
						base_collision_fn,
						motion_check_visualization_fn,
						equal_weights_distance,
						100,
						accept_at
					);
				});
		}
	});

	viewer.addTimerCallback([&]() {
		run_queue->run_all(viewer);
		throttle.allow_advance();
	});

	viewer.start();
}

/**
 * Computes the camera position for a given target point.
 * The camera is positioned at a fixed distance and height from the target,
 * with a rotation around the Z-axis.
 *
 * @param target The target point to focus the camera on.
 * @return The computed camera position.
 */
math::Vec3d camera_position_for_point(const math::Vec3d &target) {
	return math::Quaterniond::fromAxisAngle(math::Vec3d::UnitZ(), M_PI * 0.2).rotate(
		       target.withZ(0).normalized() * 10.0).withZ(5.0) + target;
}

/**
 * Finds the nearest point and normal on a convex hull shell of a tree model.
 *
 * @param target The target point to which the nearest shell point and normal are computed.
 * @param tree The tree model data containing the shell mesh.
 * @return A structure containing the nearest surface point and its normal.
 */
cgal::SurfacePointAndNormal nearest_shell_point_and_normal(
	const math::Vec3d &target,
	const cgal::CgalMeshData &tree) {
	return from_face_location(locate_nearest(target, tree), tree);
}

/**
 * Finds the nearest shell configuration/state to a target point.
 *
 * @param target The target point to which the nearest shell state is computed.
 * @param tree The tree model data containing the shell mesh.
 * @param robot The robot model used for computing the shell state.
 * @return The computed ideal shell state.
 */
RobotState compute_ideal_shell_state(const math::Vec3d &target,
                                     const cgal::CgalMeshData &tree,
                                     const robot_model::RobotModel &robot) {
	const auto surface_pt = nearest_shell_point_and_normal(target, tree);
	return fromEndEffectorAndVector(robot, surface_pt.surface_point, surface_pt.normal);
}

/**
 * Visualization of the straight-in approach planning method. That is, a method that
 * just tries to go straight to the goal from the nearest shell configuration.
 */
REGISTER_VISUALIZATION(straight_in) {
	auto robot = experiments::createStraightArmRobotModel(1.0);
	auto tree = experiments::loadBenchmarkTreemodelData("appletree");

	viewer.addTree(tree.tree_mesh);
	visualization::visualize(viewer, *tree.tree_convex_hull);

	ThrottledRunQueue rq;

	auto base_collision_fn =
			collision_check_fn_in_environment(
				{robot, *tree.tree_collision_object});

	auto motion_check_fn =
			create_motion_check_visualization_fn(
				base_collision_fn,
				rq,
				robot);

	auto start_time = std::chrono::high_resolution_clock::now();

	std::thread algorithm_thread([&]() {
		while (true) {
			for (const auto &target: tree.target_points) {
				rq.run_main_void([&](SimpleVtkViewer &viewer) {
					viewer.setCameraTransform(camera_position_for_point(target), target);
				});
				rq.throttle.wait_and_advance(30);

				// Find a nearby shell state:
				RobotState ideal_shell_state = compute_ideal_shell_state(target, *tree.tree_convex_hull, robot);

				auto actor = rq.run_main<RobotActors>([&](SimpleVtkViewer &viewer) {
					return vizualize_robot_state(viewer, robot, ideal_shell_state, {1.0, 1.0, 1.0});
				});
				rq.throttle.wait_and_advance(30);
				rq.run_main_void([&](SimpleVtkViewer &viewer) {
					for (auto &a: actor.actors) {
						viewer.removeActor(a);
					}
				});

				// Project it onto the goal:
				RobotState goal_state = project_to_goal(
					robot,
					ideal_shell_state,
					robot.findLinkByName("flying_base"),
					robot.findLinkByName("end_effector"),
					target);

				// Check if the motion collides:
				if (!motion_check_fn(ideal_shell_state, goal_state)) {
					std::vector<RobotActors> actors;
					rq.run_main_void([&](SimpleVtkViewer &viewer) {
						for (int i = 0; i <= 10; ++i) {
							double t = i / 10.0;
							RobotState intermediate = interpolate(ideal_shell_state, goal_state, t);
							actors.push_back(vizualize_robot_state(viewer, robot, intermediate, {1.0, 1.0, 0.0}));
						}
					});
					rq.throttle.wait_and_advance(30);
					rq.run_main_void([&](SimpleVtkViewer &viewer) {
						for (auto &actor: actors) {
							for (auto &a: actor.actors) {
								viewer.removeActor(a);
							}
						}
					});
				} else {
					std::vector<RobotActors> actors;
					rq.run_main_void([&](SimpleVtkViewer &viewer) {
						for (int i = 0; i <= 10; ++i) {
							double t = i / 10.0;
							RobotState intermediate = interpolate(ideal_shell_state, goal_state, t);
							actors.push_back(vizualize_robot_state(viewer, robot, intermediate, {1.0, 0.0, 0.0}));
						}
					});
					rq.throttle.wait_and_advance(30);
					rq.run_main_void([&](SimpleVtkViewer &viewer) {
						for (auto &actor: actors) {
							for (auto &a: actor.actors) {
								viewer.removeActor(a);
							}
						}
					});
				}

				rq.run_main_void([&](SimpleVtkViewer &viewer) {
					// If we've been recording for 1 minute, stop here.
					if (viewer.isRecording() && std::chrono::high_resolution_clock::now() - start_time >
					    std::chrono::minutes(1)) {
						viewer.stop();
					}
				});
			}
		}
	});

	viewer.addTimerCallback([&]() {
		rq.run_queue.run_all(viewer);
		rq.throttle.allow_advance();
	});

	viewer.lockCameraUp();

	viewer.start();
}

