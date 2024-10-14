module;

#include <memory>
#include <vector>
#include <functional>
#include <vtkActor.h>

#include "Throttle.h"
#include "RunQueue.h"
#include "robot_state.h"
#include "SimpleVtkViewer.h"
#include "../math/Vec3.h"
#include "../planning/RobotModel.h"
#include "../visualization/robot_state.h"

// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

export module collision_visualization;

import collision_detection;

using namespace mgodpl;
using namespace visualization;
using namespace vizualisation;
using namespace robot_model;

export namespace mgodpl::visualization {

	/**
	 * @brief Creates a visualized collision checking function, wrapping the given base collision checking function.
	 *
	 * @param base_collision_fn 	A function that checks for collisions in a given robot state.
	 * @param actors 				A vector to store the visualized robot actors.
	 * @param run_queue 			A queue to manage visualization tasks.
	 * @param throttle 				A throttle to control the rate of visualization updates.
	 * @param robot 				The robot model used for visualization and collision checking.
	 *
	 * @return 						A function that takes a RobotState and returns a boolean indicating if the state is in collision.
	 */
	CollisionDetectionFn state_collision_check_fn(
			const CollisionDetectionFn &base_collision_fn,
			std::shared_ptr<std::vector<RobotActors>> actors,
			std::shared_ptr<RunQueue> run_queue,
			Throttle &throttle,
			const RobotModel &robot) {

		return [&base_collision_fn, &throttle, actors, robot, run_queue](const RobotState &state) {
			// Check if the given state is in collision
			bool collides = base_collision_fn(state);

			// Enqueue a task to visualize the robot state
			run_queue->enqueue([&actors, &robot, state, collides](SimpleVtkViewer &viewer) {
				actors->push_back(vizualize_robot_state(viewer, robot, forwardKinematics(robot, state),
														collides ? math::Vec3d(1.0, 0.0, 0.0) : math::Vec3d(0.0,
																											1.0,
																											0.0)));
			});

			// Wait for the throttle to allow the next step
			throttle.wait_and_advance(1);
			return collides;
		};
	}

	/**
	 * @brief Pauses the execution and cleans up the previous actors from the viewer.
	 *
	 * @param throttle The throttle to control the rate of visualization updates.
	 * @param run_queue The queue to manage visualization tasks.
	 * @param actors The vector of robot actors to be cleaned up.
	 */
	void pause_and_cleanup(Throttle &throttle, RunQueue &run_queue, std::vector<RobotActors> &actors) {
		for (int i = 0; i < 10; ++i) {
			throttle.wait_and_advance(1);
		}

		run_queue.enqueue([&actors](SimpleVtkViewer &viewer) {
			// Clear the previous actors:
			for (auto &actor: actors) {
				for (auto &a: actor.actors) {
					viewer.removeActor(a);
				}
			}
		});
	}

	/**
	 * @brief Creates a motion collision visualization function.
	 *
	 * This function checks for collisions between two robot states and visualizes the process.
	 * It uses a state collision function to check for collisions at sampled intervals along the motion path.
	 * The function also pauses and cleans up previous visualizations.
	 *
	 * @param base_collision_fn A function that checks for collisions in a given robot state.
	 * @param actors A shared pointer to a vector storing the visualized robot actors.
	 * @param run_queue A shared pointer to a queue managing visualization tasks.
	 * @param throttle A throttle to control the rate of visualization updates.
	 * @param robot The robot model used for visualization and collision checking.
	 * @return A function that takes two RobotState objects and returns a boolean indicating if the motion is in collision.
	 */
	MotionCollisionDetectionFn create_motion_check_visualization_fn(
			const CollisionDetectionFn &base_collision_fn,
			std::shared_ptr<std::vector<RobotActors>> actors,
			std::shared_ptr<RunQueue> run_queue,
			Throttle &throttle,
			const RobotModel &robot) {

		auto check_state = state_collision_check_fn(base_collision_fn, actors, run_queue, throttle, robot);
		auto check_motion = motion_collision_check_fn_from_state_collision(check_state);

		return [&throttle, run_queue, actors, check_motion](const RobotState &from, const RobotState &to) {
			bool collides = check_motion(from, to);
			pause_and_cleanup(throttle, *run_queue, *actors);
			return collides;
		};
	}

	/**
	 * @brief A single-sample collision visualization function.
	 *
	 * This creates a CollisionDetectionFn that wraps the given base collision checking function,
	 * visualizing the state briefly and then cleaning up the visualization.
	 *
	 * @param base_collision_fn 	A function that checks for collisions in a given robot state.
	 * @param run_queue 			A shared pointer to a queue managing visualization tasks.
	 * @param throttle 				A throttle to control the rate of visualization updates.
	 * @param actors 				A shared pointer to a vector storing the visualized robot actors.
	 * @param robot 				The robot model used for visualization and collision checking.
	 * @return A function that takes a RobotState and returns a boolean indicating if the state is in collision.
	 */
	CollisionDetectionFn visualize_and_cleanup_state(
			const CollisionDetectionFn &base_collision_fn,
			std::shared_ptr<RunQueue> run_queue,
			Throttle &throttle,
			const robot_model::RobotModel &robot) {

		// Create a new collision detection function that visualizes the state and cleans up the visualization
		return [base_collision_fn, run_queue, &throttle, &robot](const RobotState &goal) {

			auto actors = std::make_shared<std::optional<RobotActors>>();

			// run the base collision function
			bool collides = base_collision_fn(goal);

			// Visualize the robot state, color it red if it collides
			run_queue->enqueue([actors, &robot, goal, collides](SimpleVtkViewer &viewer) {
				*actors = (vizualize_robot_state(viewer, robot, forwardKinematics(robot, goal),
												 collides ? math::Vec3d(1.0, 0.0, 0.0) : math::Vec3d(0.0,
																									 1.0,
																									 0.0)));
			});

			throttle.wait_and_advance(collides ? 5 : 20);

			// Remove it again:
			run_queue->enqueue([actors](SimpleVtkViewer &viewer) {
				if (actors->has_value()) {
					for (auto &a: actors->value().actors) {
						viewer.removeActor(a);
					}
				}
			});

			return collides;
		};
	}
}