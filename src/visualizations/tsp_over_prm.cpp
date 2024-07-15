// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

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
#include "../visualization/VtkLineSegmentVizualization.h"
#include "../experiment_utils/declarative/fruit_models.h"
#include <fcl/narrowphase/collision.h>
#include <boost/graph/adjacency_list.hpp>

#include "../planning/collision_detection.h"
#include "../planning/goal_sampling.h"
#include "../visualization/declarative.h"

using namespace mgodpl;

/**
 * A variant of Probabilitsic Roadmap (PRM), designed to create a base "infrastructure" roadmap, and then connect up an arbitrary number of goals to this roadmap.
 *
 * It works as follows:
 *
 * First, it iteratively:
 * - Samples a random state.
 * - Checks if it collides with the environment.
 * - If it collides, the state is discarded.
 * - Otherwise:
 *	 - The state is added to the roadmap as an "infrastructure" node.
 *	 - The k nearest neighbors are looked up.
 *	 - For each neighbor, if a collision-free motion exists between the new state and the neighbor, an edge is added.
 *
 * The above process is repeated until the maximum number of samples is reached.
 *
 * Then, for each goal:
 *	- A random goal state is sampled.
 *	- If it collides with the environment, it is discarded.
 *	- Otherwise:
 *	  - The state is added to the roadmap as a "goal" node.
 *	  - The k nearest neighbors are looked up; only infrastructure nodes are considered.
 *	  - For each neighbor, if a collision-free motion exists between the new state and the neighbor, an edge is added.
 */
struct TwoTierMultigoalPRM {
	/**
	 * @brief The properties of a vertex in the PRM as stored by Boost Graph Library.
	 */
	struct VertexProperties {
		RobotState state;
	};

	// Define the graph type: an undirected graph with the defined vertex properties
	using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperties>;

	// The number of nearest neighbors to connect to. (This is more like traditional PRM, rather than PRM* which uses a dynamic number of neighbors.)
	size_t n_neighbours = 5;

	// The Boost Graph representing the roadmap.
	Graph graph;

	// A vector of all the infrastructure nodes. (TODO: make this a spatial index instead.)
	std::vector<Graph::vertex_descriptor> infrastructure_nodes;

	// A function that checks if a motion between two states collides.
	std::function<bool(const RobotState &, const RobotState &)> check_motion_collides;

	/**
	 * @brief Construct a new TwoTierMultigoalPRM object, representing an empty roadmap.
	 *
	 * @param n_neighbours				The number of nearest neighbors to connect to.
	 * @param check_motion_collides		A function that checks if a motion between two states collides.
	 */
	TwoTierMultigoalPRM(size_t n_neighbours,
	                    std::function<bool(const RobotState &, const RobotState &)> check_motion_collides)
		: n_neighbours(n_neighbours),
		  check_motion_collides(std::move(check_motion_collides)) {
	}

	/**
	 * This function looks up the k nearest neighbors to a given state. These are infrastructure nodes only.
	 *
	 * @param state		The state to find the neighbors for; it need not yet be in the graph.
	 * @return			A vector of the k nearest neighbors, paired with their distances. (This may be less than k if there are fewer nodes in the graph.)
	 */
	std::vector<std::pair<double, Graph::vertex_descriptor> > k_nearest_neighbors(const RobotState &state) {
		// Keep the distances and the indices.
		std::vector<std::pair<double, Graph::vertex_descriptor> > distances;
		distances.reserve(n_neighbours + 1);
		// Reserve k+1 elements, so we can insert the new element at the correct position.

		// Iterate over all infrastructure nodes (TODO: do better than this linear search.)
		for (auto v: infrastructure_nodes) {
			// Get the properties of the node (i.e., the state).
			const VertexProperties &node = graph[v];

			bool inserted = false;

			// Iterate over all neighbors found so far.
			for (size_t i = 0; i < distances.size(); ++i) {
				// If the distance is smaller than the current distance, insert it.
				if (distances[i].first > equal_weights_distance(node.state, state)) {
					distances.insert(distances.begin() + i, {equal_weights_distance(node.state, state), v});

					// Mark that we've inserted the element.
					inserted = true;

					// If it's now larger than n_neighbours, remove the last element.
					if (distances.size() > n_neighbours) {
						distances.pop_back();
					}
					break; // Once we've inserted, we're done.
				}
			}

			// If we haven't inserted it yet, and there's still space, add it to the end.
			if (!inserted && distances.size() < n_neighbours) {
				distances.emplace_back(equal_weights_distance(node.state, state), v);
			}
		}

		return distances;
	}

	/**
	 * @brief Add a new node to the PRM, connecting it up to the nearest neighbors.
	 *
	 * @param	state	The state of the new node.
	 * @param	is_goal_sample	Whether this is a goal sample or not (if it is, it will not be added to the infrastructure nodes.)
	 * @returns The vertex descriptor of the new node.
	 */
	Graph::vertex_descriptor add_roadmap_node(const RobotState &state, bool is_goal_sample = false) {
		// Find the k nearest neighbors. (Brute-force; should migrate to a VP-tree when I can.)
		auto k_nearest = k_nearest_neighbors(state);

		// Add the new node to the graph.
		auto new_vertex = boost::add_vertex({state}, graph);

		// Then add the edges:
		for (const auto &[distance, neighbor]: k_nearest) {
			// Do collision check: if it collides, don't add the edge.
			if (check_motion_collides(graph[neighbor].state, state)) {
				continue;
			}

			boost::add_edge(new_vertex, neighbor, graph);
		}

		// If it's not a goal sample, add it to the infrastructure nodes.
		if (!is_goal_sample) {
			infrastructure_nodes.push_back(new_vertex);
		}

		// Return the graph vertex id.
		return new_vertex;
	}
};


REGISTER_VISUALIZATION(tsp_over_prm) {
	// Look up the tree model; force the camera to stay upright.
	viewer.lockCameraUp();
	viewer.setCameraTransform({20, 10, 8}, {0, 0, 5});

	// Create a random number generator; seeded for reproducibility.
	random_numbers::RandomNumberGenerator rng(42);

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

	// Max 100 samples (bit small, but avoids visual clutter).
	const size_t max_samples = 100;

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

	// Allocate an empty prm.
	TwoTierMultigoalPRM prm(5,
	                        [&](const RobotState &a, const RobotState &b) {
		                        return check_motion_collides(robot, tree_collision, a, b);
	                        });

	// The actors for the last-vizualized robot configuration sample, so we can remove them later.
	std::optional<vizualisation::RobotActors> sample_viz;

	// Index to track which goal to sample next in the visualization.
	size_t goal_being_sampled = 0;

	// How many times to sample per goal.
	const size_t max_samples_per_goal = 2;

	// Look up the link IDs for the base and end effector.
	robot_model::RobotModel::LinkId base_link = robot.findLinkByName("flying_base");
	robot_model::RobotModel::LinkId end_effector_link = robot.findLinkByName("end_effector");

	// Finally, register our timer callback.
	viewer.addTimerCallback([&]() {
		// Some slow-down logic for visualization.
		if (frames_until_sample > 0) {
			--frames_until_sample;
			return;
		}
		frames_until_sample = 1;

		// Remove the last sample visualization, if it exists.
		if (sample_viz) {
			for (const auto &vtk_actor: sample_viz->actors) {
				viewer.viewerRenderer->RemoveActor(vtk_actor);
			}
		}

		// Execute different logic depending on whether the infrastructure nodes are full or not.
		if (boost::num_vertices(prm.graph) < max_samples) {

			// Generate a random state.
			auto state = generateUniformRandomState(robot, rng, 5.0, 10.0);

			// Check if the robot collides with the tree.
			bool collides = check_robot_collision(robot, tree_collision, state);

			// Pick a color based on whether it collides.
			math::Vec3d color = collides
				                    ? math::Vec3d{1.0, 0.0, 0.0}
				                    : // Red if it collides.
				                    math::Vec3d{0.0, 1.0, 0.0}; // Green if it doesn't.

			// Visualize the robot state.
			sample_viz = vizualisation::vizualize_robot_state(viewer,
			                                                  robot,
			                                                  robot_model::forwardKinematics(
				                                                  robot,
				                                                  state.joint_values,
				                                                  0,
				                                                  state.base_tf),
			                                                  color);

			// If this sample collides, don't add it to the roadmap.
			if (collides) {
				return;
			}

			// Otherwise, add it to the roadmap, and try to connect it to the nearest neighbors.
			auto new_vertex = prm.add_roadmap_node(state, false);

			// Iterate over the graph vertex neighbors and add the edges to the visualization.
			for (const auto &neighbor: boost::make_iterator_range(boost::adjacent_vertices(new_vertex, prm.graph))) {
				edges.emplace_back(
					prm.graph[new_vertex].state.base_tf.translation,
					prm.graph[neighbor].state.base_tf.translation
				);
			}

			prm_edges.updateLine(edges);
		} else if (goal_being_sampled < fruit_positions.size()) {
			size_t samples_found = 0;

			for (int attempt = 0; attempt < 100; ++attempt) {
				auto goal_state = genGoalStateUniform(
					rng,
					fruit_positions[goal_being_sampled],
					robot,
					base_link,
					end_effector_link
				);

				// Check collisions:
				if (!check_robot_collision(robot, tree_collision, goal_state)) {
					samples_found += 1;

					sample_viz = vizualisation::vizualize_robot_state(viewer,
					                                                  robot,
					                                                  robot_model::forwardKinematics(
						                                                  robot,
						                                                  goal_state.joint_values,
						                                                  0,
						                                                  goal_state.base_tf),
					                                                  {0.0, 0.0, 1.0});

					// Add the goal to the roadmap.
					auto new_vertex = prm.add_roadmap_node(goal_state, true);

					// Add the links:
					for (const auto &neighbor: boost::make_iterator_range(
						     boost::adjacent_vertices(new_vertex, prm.graph))) {
						goal_edges.emplace_back(
							prm.graph[new_vertex].state.base_tf.translation,
							prm.graph[neighbor].state.base_tf.translation
						);
					}

					prm_goal_edges.updateLine(goal_edges);

					if (samples_found >= max_samples_per_goal) {
						break;
					}
				}
			}

			goal_being_sampled += 1;
		} else {
			viewer.stop();
		}
	});

	viewer.start();
}
