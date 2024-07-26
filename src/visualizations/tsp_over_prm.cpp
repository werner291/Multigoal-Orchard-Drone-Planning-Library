// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <condition_variable>
#include <vtkRenderer.h>
#include <fcl/narrowphase/collision_object.h>

#include "../experiment_utils/default_colors.h"
#include "../visualization/visualization_function_macros.h"
#include "../visualization/robot_state.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../planning/RandomNumberGenerator.h"
#include "../planning/RobotState.h"
#include "../planning/state_tools.h"
#include "../planning/vptree.hpp"
#include "../planning/fcl_utils.h"
#include "../planning/traveling_salesman.h"
#include "../planning/tsp_over_prm.h"
#include "../visualization/VtkLineSegmentVizualization.h"
#include "../visualization/ladder_trace.h"
#include "../visualization/Throttle.h"
#include "../experiment_utils/declarative/fruit_models.h"
#include <fcl/narrowphase/collision.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>

#include "../planning/collision_detection.h"
#include "../planning/goal_sampling.h"
#include "../visualization/declarative.h"

using namespace mgodpl;

REGISTER_VISUALIZATION(tsp_over_prm) {
	using namespace mgodpl;
	using namespace visualization;

	// Look up the tree model; force the camera to stay upright.
	viewer.lockCameraUp();
	viewer.setCameraTransform({20, 10, 8}, {0, 0, 5});

	// Create a random number generator; seeded for reproducibility.
	random_numbers::RandomNumberGenerator rng(42);

	// We're going to "throttle" the algorithm: it's going to run in a separate thread, but only advance when it's allowed to.
	Throttle throttle;

	// A mutex for the algorithm thread to write to the to-be-visualized data.
	std::mutex data_transfer_mutex;

	/**
	 * A vector of recently-sampled states, paired with a boolean indicating whether they were added to the roadmap.
	 * These should be visualized on the next frame, and cleared the frame after that.
	 * Note: access to this vector should be protected by the data_transfer_mutex.
	 */
	std::vector<std::pair<RobotState, bool> > recent_samples;
	std::optional<RobotPath> final_path;
	PathPoint final_path_point = {0, 0};
	std::optional<vizualisation::RobotActors> final_path_follower;

	// Create a tree model.
	experiments::TreeModelCache cache;
	auto tree = declarative::instantiate_tree_model(
		declarative::TreeModelParameters{
			.name = "appletree",
			.leaf_scale = 1.0,
			.fruit_subset = declarative::Unchanged{},
			.seed = 42
		},
		cache,
		rng);
	// Extract the fruit positions from the tree.
	const auto fruit_positions = fruit_positions_from_models(tree.fruit_models);

	// Add the tree to the visualization.
	visualization::visualize(viewer, tree);

	// Create a procedural robot model.
	auto robot = experiments::createProceduralRobotModel();

	// Define a start state somewhere outside the tree...
	RobotState start_state;
	start_state.joint_values = std::vector(robot.count_joint_variables(), 0.0);
	start_state.base_tf = math::Transformd::fromTranslation({-10, -10, 0});

	// Max 100 samples (bit small, but avoids visual clutter).
	const size_t max_samples = 100;
	const size_t max_samples_per_goal = 1;
	const size_t n_neighbours = 5;

	// Initialize a visualization for the edges.
	VtkLineSegmentsVisualization prm_edges(1, 0, 1);
	std::vector<std::pair<math::Vec3d, math::Vec3d> > edges;
	viewer.addActor(prm_edges.getActor());

	// And the non-infrastructure edges.
	VtkLineSegmentsVisualization prm_goal_edges(0.5, 1.0, 0.5);
	std::vector<std::pair<math::Vec3d, math::Vec3d> > goal_edges;
	viewer.addActor(prm_goal_edges.getActor());

	// Add a frame counter so we can slow down the sampling for visualization.
	int frames_until_sample = 0;

	// Create a collision object BVH for the tree trunk.
	fcl::CollisionObjectd tree_collision(fcl_utils::meshToFclBVH(tree.tree_model->meshes.trunk_mesh));

	// The actors for the last-vizualized robot configuration sample(s), so we can remove them later.
	std::vector<vizualisation::RobotActors> sample_viz;

	// Look up the link IDs for the base and end effector.
	robot_model::RobotModel::LinkId base_link = robot.findLinkByName("flying_base");
	robot_model::RobotModel::LinkId end_effector_link = robot.findLinkByName("end_effector");

	VtkLineSegmentsVisualization distance_edges_viz(0.5, 0.5, 0.5);
	viewer.addActor(distance_edges_viz.getActor());

	std::function viz_sampled_state = [&](const RobotState &state, bool added) {
		{
			std::lock_guard lock(data_transfer_mutex);
			recent_samples.emplace_back(state, added);
		}
		throttle.wait_and_advance();
	};

	// A callback for visualizing a single edge.
	std::function viz_edge = [&](
		std::pair<const RobotState &, const PRMGraph::vertex_descriptor &> source,
		std::pair<const RobotState &, const PRMGraph::vertex_descriptor &> target,
		bool added) {
		if (added) {
			{
				std::lock_guard lock(data_transfer_mutex);

				edges.emplace_back(
					source.first.base_tf.translation,
					target.first.base_tf.translation
				);
			}
			throttle.wait_and_advance();
		}
	};

	std::function viz_goal_edge = [&](
		std::pair<const RobotState &, const PRMGraph::vertex_descriptor &> source,
		std::pair<const RobotState &, const PRMGraph::vertex_descriptor &> target,
		bool added) {
		if (added) {
			{
				std::lock_guard lock(data_transfer_mutex);
				goal_edges.emplace_back(
					source.first.base_tf.translation,
					target.first.base_tf.translation
				);
			}
			throttle.wait_and_advance();
		}
	};

	PrmBuildHooks infrastructure_sample_hooks{
		.on_sample = viz_sampled_state,
		.add_roadmap_node_hooks = AddRoadmapNodeHooks{
			.on_edge_considered = viz_edge
		}
	};

	GoalSampleHooks goal_sampling_hooks{
		.on_sample = viz_sampled_state,
		.add_roadmap_node_hooks = AddRoadmapNodeHooks{
			.on_edge_considered = viz_goal_edge
		}
	};

	TspOverPrmHooks hooks{
		.infrastructure_sample_hooks = infrastructure_sample_hooks,
		.goal_sample_hooks = goal_sampling_hooks
	};

	std::thread algorithm_thread([&]() {
		// Plan the path.
		auto path = plan_path_tsp_over_prm(
			start_state,
			fruit_positions,
			robot,
			tree_collision,
			TspOverPrmParameters{
				.n_neighbours = n_neighbours,
				.max_samples = max_samples,
				.goal_sample_params = GoalSampleParams{
					.k_neighbors = n_neighbours,
					.max_valid_samples = max_samples_per_goal,
					.max_attempts = 100
				}
			},
			rng,
			hooks
		);

		// Visualize the path.
		{
			std::lock_guard lock(data_transfer_mutex);
			final_path = path;
		}
	});

	// Finally, register our timer callback.
	viewer.addTimerCallback([&]() {
			// Some slow-down logic for visualization.
			if (frames_until_sample > 0) {
				--frames_until_sample;
				return;
			}
			frames_until_sample = 1;

			// Remove the last sample visualization, if it exists.
			for (const auto &vtk_actors: sample_viz) {
				for (const auto &vtk_actor: vtk_actors.actors) {
					viewer.viewerRenderer->RemoveActor(vtk_actor);
				}
			}

			// Process updates received from the algorithm thread.
			{
				// Lock the mutex.
				std::lock_guard lock(data_transfer_mutex);

				// Visualize any states we received.
				for (const auto &[state, added]: recent_samples) {
					// Pick a color based on whether it collides.
					math::Vec3d color = added ? math::Vec3d{0.0, 1.0, 0.0} : math::Vec3d{1.0, 0.0, 0.0};

					// Compute the forward kinematics.
					auto fk = forwardKinematics(robot, state.joint_values, 0, state.base_tf);

					// Visualize the robot state.
					sample_viz.push_back(vizualisation::vizualize_robot_state(viewer, robot, fk, color));
				}

				// update the edges:
				prm_edges.updateLine(edges);
				prm_goal_edges.updateLine(goal_edges);

				// If there's a path, vizualize it as a ladder trace.
				if (final_path.has_value()) {
					if (!final_path_follower) {
						visualize_ladder_trace(robot, *final_path, viewer);

						// Set up the final path follower.
						final_path_follower = vizualisation::vizualize_robot_state(
							viewer,
							robot,
							robot_model::forwardKinematics(robot, start_state.joint_values, 0, start_state.base_tf),
							{1, 0, 0});
					} else {
						// Advance the path point:
						final_path_point = final_path_point.adjustByScalar(0.01, *final_path);

						// Update the state.
						RobotState state = interpolate(final_path_point, *final_path);

						update_robot_state(robot,
						                   forwardKinematics(
							                   robot,
							                   state.joint_values,
							                   0,
							                   state.base_tf),
						                   *final_path_follower);
					}
				}

				recent_samples.clear();
			}

			throttle.allow_advance();
		}

	);

	viewer.start();
}
