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

struct VertexProperties {
	RobotState state;
};

// Define the graph type: an undirected graph with the defined vertex properties
using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperties>;

struct PRM {
	size_t n_neighbours = 5;

	Graph graph;

	std::function<bool(const RobotState &, const RobotState &)> check_motion_collides;

	std::vector<Graph::vertex_descriptor>
	k_nearest_neighbors_lineartime(const RobotState &state, size_t k) {
		// Keep the distances and the indices.
		std::vector<std::pair<double, Graph::vertex_descriptor> > distances;
		distances.reserve(k + 1);

		// Iterate over all nodes.
		auto [vertices_begin, vertices_end] = boost::vertices(graph);
		for (auto vit = vertices_begin; vit != vertices_end; ++vit) {
			const VertexProperties &node = graph[*vit];

			bool inserted = false;

			// Iterate over all neighbors found so far.
			for (size_t i = 0; i < distances.size(); ++i) {
				// If the distance is smaller than the current distance, insert it.
				if (distances[i].first > equal_weights_distance(node.state, state)) {
					distances.insert(distances.begin() + i, {equal_weights_distance(node.state, state), *vit});
					inserted = true;
					// If it's now larger than n_neighbours, remove the last element.
					if (distances.size() > k) {
						distances.pop_back();
					}
					break;
				}
			}

			if (!inserted && distances.size() < k) {
				distances.emplace_back(equal_weights_distance(node.state, state), *vit);
			}
		}

		std::vector<Graph::vertex_descriptor> result;
		result.reserve(k);
		for (const auto &d: distances) {
			result.push_back(d.second);
		}

		return result;
	}


	/**
	 * @brief Add a new node to the PRM, connecting it up to the nearest neighbors.
	 *
	 * @param	state	The state of the new node.
	 * @returns The vertex descriptor of the new node.
	 */
	Graph::vertex_descriptor add_node(const RobotState &state) {
		// Find the k nearest neighbors. (Brute-force; should migrate to a VP-tree when I can.)
		auto k_nearest = k_nearest_neighbors_lineartime(state, n_neighbours);

		// Add the new node to the graph.
		auto new_vertex = boost::add_vertex({state}, graph);

		// Then add the edges:
		for (const auto &neighbor: k_nearest) {
			// Do collision check: if it collides, don't add the edge.
			if (check_motion_collides(graph[neighbor].state, state)) {
				continue;
			}

			boost::add_edge(new_vertex, neighbor, graph);
		}

		return new_vertex;
	}
};


REGISTER_VISUALIZATION(tsp_over_prm) {
	// Create a random number generator.
	random_numbers::RandomNumberGenerator rng;

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

	visualization::visualize(viewer, tree);

	auto robot = experiments::createProceduralRobotModel();


	const size_t max_samples = 100;

	// Initialize a visualization for the edges.
	VtkLineSegmentsVisualization prm_edges(1, 0, 1);
	std::vector<std::pair<math::Vec3d, math::Vec3d> > edges;
	viewer.addActor(prm_edges.getActor());

	// Add a frame counter so we can slow down the sampling for visualization.
	int frames_until_sample = 0;

	// Create a collision object BVH for the tree trunk.
	fcl::CollisionObjectd tree_collision(fcl_utils::meshToFclBVH(tree.tree_model->meshes.trunk_mesh));

	// Allocate an empty prm.
	PRM prm{
		.n_neighbours = 5,
		.graph = Graph(),
		.check_motion_collides = [&](const RobotState &a, const RobotState &b) {
			return check_motion_collides(robot, tree_collision, a, b);
		}
	};

	// The actors for the last-vizualized robot configuration sample, so we can remove them later.
	std::optional<vizualisation::RobotActors> sample_viz;

	size_t goal_being_sampled = 0;

	const size_t max_samples_per_goal = 1;

	const auto fruit_positions = fruit_positions_from_models(tree.fruit_models);

	robot_model::RobotModel::LinkId base_link = robot.findLinkByName("flying_base");
	robot_model::RobotModel::LinkId end_effector_link = robot.findLinkByName("end_effector");

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


		// Skip if we're at the maximum number of samples, and stop the viewer.
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
			auto new_vertex = prm.add_node(state);

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

					vizualisation::vizualize_robot_state(viewer,
					                                     robot,
					                                     robot_model::forwardKinematics(
						                                     robot,
						                                     goal_state.joint_values,
						                                     0,
						                                     goal_state.base_tf),
					                                     {0.0, 0.0, 1.0});

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
