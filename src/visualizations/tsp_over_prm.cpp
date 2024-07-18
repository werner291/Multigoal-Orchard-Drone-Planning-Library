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
#include "../planning/traveling_salesman.h"
#include "../visualization/VtkLineSegmentVizualization.h"
#include "../visualization/ladder_trace.h"
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
		/// The state of the robot at this vertex.
		RobotState state;
		/// The index of the goal this vertex represents, if it is a goal vertex.
		/// The index is in two parts: the first part is the fruit index, the second part is the goal sample index.
		std::optional<std::pair<size_t, size_t> > goal_index;
	};

	// Define the graph type: an undirected graph with the defined vertex properties
	using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperties, boost::property<
		boost::edge_weight_t, double> >;

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
	Graph::vertex_descriptor add_roadmap_node(const RobotState &state,
	                                          std::optional<std::pair<size_t, size_t> > goal_index) {
		// Find the k nearest neighbors. (Brute-force; should migrate to a VP-tree when I can.)
		auto k_nearest = k_nearest_neighbors(state);

		// Add the new node to the graph.
		auto new_vertex = boost::add_vertex({state, goal_index}, graph);

		// Then add the edges:
		for (const auto &[distance, neighbor]: k_nearest) {
			// Do collision check: if it collides, don't add the edge.
			if (check_motion_collides(graph[neighbor].state, state)) {
				continue;
			}

			boost::add_edge(new_vertex, neighbor, distance, graph);
		}

		// If it's not a goal sample, add it to the infrastructure nodes.
		if (!goal_index) {
			infrastructure_nodes.push_back(new_vertex);
		}

		// Return the graph vertex id.
		return new_vertex;
	}
};

/**
 * This struct provides a table of indices for goal samples, grouped by fruit.
 * It also tracks the total number of samples.
 *
 * Specifically, it allows to look up two things:
 * - Given a fruit index, the indices of the goal samples for that fruit.
 * - Given a fruit index and a goal sample index, the goal sample index in the global list of goal samples.
 *
 * Note: these are NOT graph vertex IDs.
 */
class GroupIndexTable {
	size_t total_samples;
	std::vector<std::vector<size_t> > index_table;

public:
	/**
	 * 
	 * @param counts
	 */
	explicit GroupIndexTable(const std::vector<size_t> &counts) : index_table(counts.size()) {
		size_t global_index = 0;

		for (size_t i = 0; i < counts.size(); ++i) {
			index_table[i].reserve(counts[i]);
			for (size_t j = 0; j < counts[i]; ++j) {
				index_table[i].push_back(global_index++);
			}
		}

		total_samples = global_index;
	}

	/**
	 * @brief Look up the indices of the goal samples for a given fruit.
	 * @param fruit_index	The index of the fruit.
	 * @return			A vector of the indices of the goal samples for that fruit.
	 */
	[[nodiscard]] const std::vector<size_t> &for_fruit(size_t fruit_index) const {
		return index_table[fruit_index];
	}

	/**
	 * @brief Look up the global index of a goal sample.
	 * @param fruit_index	The index of the fruit.
	 * @param sample_index	The index of the goal sample within the fruit.
	 * @return		The global index of the goal sample.
	 */
	[[nodiscard]] const size_t &lookup(size_t fruit_index, size_t sample_index) const {
		return index_table[fruit_index][sample_index];
	}

	/**
	 * @brief Get the total number of goal samples.
	 */
	[[nodiscard]] inline size_t total() const {
		return total_samples;
	}
};

/**
 * @brief Run Dijkstra's algorithm on a graph, starting from a given node.
 *
 * This function abstracts over Boost, running Dijkstra on a (PRM) graph, returning the distance map and the predecessor map from the given start node.
 *
 * The distance map contains the distance to each node from the start node.
 * The predecessor map contains, for every node, the predecessor node on the shortest path from the start node.
 *
 * @param graph			The graph to run Dijkstra on.
 * @param start_node	The node to start from.
 * @return				A pair of vectors: the first vector contains the distances to each node, the second vector contains the predecessor node for each node.
 */
std::pair<std::vector<double>, std::vector<TwoTierMultigoalPRM::Graph::vertex_descriptor> >
runDijkstra(const TwoTierMultigoalPRM::Graph &graph,
            TwoTierMultigoalPRM::Graph::vertex_descriptor start_node) {
	std::vector<TwoTierMultigoalPRM::Graph::vertex_descriptor> predecessors(num_vertices(graph));
	std::vector<double> distances(num_vertices(graph));

	dijkstra_shortest_paths(graph,
	                        start_node,
	                        boost::predecessor_map(
		                        boost::make_iterator_property_map(predecessors.begin(),
		                                                          get(boost::vertex_index, graph)))
	                        .distance_map(
		                        boost::make_iterator_property_map(distances.begin(), get(boost::vertex_index, graph))));

	return {distances, predecessors};
}

/**
 * @brief Retrace a path through the predecessor map.
 *
 * This function takes a predecessor map and a goal node, and reconstructs the path from the start node to the goal node.
 *
 * @param	graph				The graph the path is in.
 * @param	predecessor_lookup	The predecessor lookup table. (The start node is implied by the structure of the table.)
 * @param   goal_node			The goal node to retrace from.
 *
 * @return	The path from the start node to the goal node. The start node is whichever node in the path has itself as a predecessor.
 */
[[nodiscard]] RobotPath retrace_path(const TwoTierMultigoalPRM::Graph &graph,
                                     const std::vector<TwoTierMultigoalPRM::Graph::vertex_descriptor> &
                                     predecessor_lookup,
                                     TwoTierMultigoalPRM::Graph::vertex_descriptor goal_node) {
	RobotPath path;

	auto current_node = goal_node;

	while (predecessor_lookup[current_node] != current_node) {
		path.states.push_back(graph[current_node].state);
		current_node = predecessor_lookup[current_node];
	}

	// Reverse the path.
	std::reverse(path.states.begin(), path.states.end());

	return path;
}

/**
 * Convert a TSP solution referring to fruit-and-sample indices to one using only global goal sample indices.
 *
 * @param group_index_table	The group index table, to translate the index pairs from the TSP solution to global goal sample
 * @param tour				The TSP solution.
 *
 * @returns			The TSP solution with global goal sample indices.
 */
std::vector<size_t> convert_tour_to_global(const GroupIndexTable &group_index_table,
                                           const std::vector<std::pair<size_t, size_t> > &tour) {
	return tour | ranges::views::transform([&](const auto &pair) {
		return group_index_table.lookup(pair.first, pair.second);
	}) | ranges::to<std::vector>();
}

/**
 * @brief Plan a visitation order based on the distance lookup tables. This is an abstraction over tsp_open_end_grouped that avoids lambdas.
 * @return The visitation order, expressed in terms of global goal sample indices.
 */
std::vector<size_t> pick_visitation_order(
	const std::vector<std::vector<double> > &distance_lookup,
	const std::vector<double> &start_to_goals_distances,
	const GroupIndexTable *group_index_table,
	const std::vector<size_t> &group_sizes
) {
	// Plan a TSP over the PRM.
	auto tour = tsp_open_end_grouped(
		[&](std::pair<size_t, size_t> a) {
			return start_to_goals_distances[group_index_table->lookup(a.first, a.second)];
		},
		[&](std::pair<size_t, size_t> a, std::pair<size_t, size_t> b) {
			return distance_lookup[group_index_table->lookup(a.first, a.second)][group_index_table->lookup(
				b.first,
				b.second)];
		},
		group_sizes
	);

	return convert_tour_to_global(*group_index_table, tour);
}

/**
 * @brief Construct the final path based on the TSP solution and predecessor lookup tables.
 *
 * @param graph							The graph that the different vertices are in.
 * @param predecessor_lookup			The predecessor lookup tables; one for each goal sample.
 * @param start_to_goals_predecessors	The predecessor lookup table for the start-to-goal samples.
 * @param goal_nodes					The graph vertices corresponding to the goal indices.
 * @param tour							The TSP solution, expressed in goal sample indices.
 *
 * @return The final path through the PRM, from the start and passing by all the goals in the TSP solution.
 */
RobotPath construct_final_path(const TwoTierMultigoalPRM::Graph &graph,
                               const std::vector<std::vector<TwoTierMultigoalPRM::Graph::vertex_descriptor> > &
                               predecessor_lookup,
                               const std::vector<TwoTierMultigoalPRM::Graph::vertex_descriptor> &
                               start_to_goals_predecessors,
                               const std::vector<TwoTierMultigoalPRM::Graph::vertex_descriptor> &goal_nodes,
                               const std::vector<size_t> &tour) {
	// Allocate a path object.
	RobotPath path;

	// Start at the start node.
	path.append(retrace_path(graph,
	                         start_to_goals_predecessors,
	                         goal_nodes[tour[0]]));

	for (size_t i = 1; i < tour.size(); ++i) {
		// Retrace the goal-to-goal path, and append it to the path.
		path.append(retrace_path(graph, predecessor_lookup[tour[i - 1]], goal_nodes[tour[i]]));
	}

	return path;
}

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

	// Define a start state somewhere outside the tree...
	RobotState start_state;
	start_state.joint_values = std::vector(robot.count_joint_variables(), 0.0);
	start_state.base_tf = math::Transformd::fromTranslation({-10, -10, 0});

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

	// Add the start state to the roadmap.
	prm.add_roadmap_node(start_state, std::nullopt);

	// The actors for the last-vizualized robot configuration sample, so we can remove them later.
	std::optional<vizualisation::RobotActors> sample_viz;

	// Index to track which goal to sample next in the visualization.
	size_t goal_being_sampled = 0;

	// How many times to sample per goal.
	const size_t max_samples_per_goal = 1;
	// Store the goal sample vertex nodes, with a separate sub-vector for each goal.
	// Start with a vector of empty vectors.
	std::vector<TwoTierMultigoalPRM::Graph::vertex_descriptor> goal_nodes;

	// A group index table, to look up the goal samples by fruit. (Optional because it's not initialized until all goals states are sampled.)
	std::optional<GroupIndexTable> group_index_table;

	// Store a goal-sample-to-goal-sample distance lookup table.
	// Given two goal samples indices i and j, distance_lookup[i][j] contains
	// the distance from goal sample i to goal sample j through the PRM.
	//
	// Note: these are global goal sample indices, not graph vertex IDs. (See group_index_table)
	std::vector<std::vector<double> > distance_lookup;
	std::vector<double> start_to_goals_distances;

	// And a predecessor lookup table to reconstruct the paths.
	std::vector<std::vector<TwoTierMultigoalPRM::Graph::vertex_descriptor> > predecessor_lookup;
	std::vector<TwoTierMultigoalPRM::Graph::vertex_descriptor> start_to_goals_predecessors;

	// Look up the link IDs for the base and end effector.
	robot_model::RobotModel::LinkId base_link = robot.findLinkByName("flying_base");
	robot_model::RobotModel::LinkId end_effector_link = robot.findLinkByName("end_effector");

	VtkLineSegmentsVisualization distance_edges_viz(0.5, 0.5, 0.5);
	viewer.addActor(distance_edges_viz.getActor());

	std::vector<size_t> group_sizes(fruit_positions.size(), 0);

	std::optional<RobotPath> unoptimized_path;

	/// The goal sample index that's being distanced and predecessor-looked-up.
	size_t next_goal_sample_index = 0;

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
				auto new_vertex = prm.add_roadmap_node(state, std::nullopt);

				// Iterate over the graph vertex neighbors and add the edges to the visualization.
				for (const auto &neighbor:
				     boost::make_iterator_range(boost::adjacent_vertices(new_vertex, prm.graph))) {
					edges.emplace_back(
						prm.graph[new_vertex].state.base_tf.translation,
						prm.graph[neighbor].state.base_tf.translation
					);
				}

				prm_edges.updateLine(edges);
			} else if (goal_being_sampled < fruit_positions.size()) {
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
						group_sizes[goal_being_sampled] += 1;

						sample_viz = vizualisation::vizualize_robot_state(viewer,
						                                                  robot,
						                                                  robot_model::forwardKinematics(
							                                                  robot,
							                                                  goal_state.joint_values,
							                                                  0,
							                                                  goal_state.base_tf),
						                                                  {0.0, 0.0, 1.0});

						// Add the goal to the roadmap.
						auto new_vertex = prm.add_roadmap_node(goal_state,
						                                       {
							                                       {
								                                       goal_being_sampled,
								                                       group_sizes[goal_being_sampled] - 1
							                                       }
						                                       });
						goal_nodes.push_back(new_vertex);

						// Add the links:
						for (const auto &neighbor: boost::make_iterator_range(
							     boost::adjacent_vertices(new_vertex, prm.graph))) {
							goal_edges.emplace_back(
								prm.graph[new_vertex].state.base_tf.translation,
								prm.graph[neighbor].state.base_tf.translation
							);
						}

						prm_goal_edges.updateLine(goal_edges);

						if (group_sizes[goal_being_sampled] >= max_samples_per_goal) {
							break;
						}
					}
				}

				goal_being_sampled += 1;

				if (goal_being_sampled == fruit_positions.size()) {
					group_index_table = GroupIndexTable(group_sizes);
				}
			} else if (next_goal_sample_index < group_index_table->total()) {
				// Run Dijkstra's algorithm on the graph, starting from the next goal sample.
				const auto &[distances, predecessors] = runDijkstra(prm.graph, goal_nodes[next_goal_sample_index]);

				// Then, filter out all the non-goal nodes.
				distance_lookup.emplace_back(goal_nodes.size());
				for (unsigned long goal_node: goal_nodes) {
					distance_lookup.back().push_back(distances[goal_node]);
				}

				// Keep the predecessor lookup table. (TODO: this might be quite memory-hungry; we'll see.)
				predecessor_lookup.push_back(predecessors);

				std::vector<std::pair<math::Vec3d, math::Vec3d> > distance_edges;
				distance_edges.reserve(predecessors.size());
				// We visualize using the predecessor map.
				for (size_t i = 0; i < predecessors.size(); ++i) {
					if (predecessors[i] != i) {
						auto origin = prm.graph[predecessors[i]].state.base_tf.translation;
						auto dest = prm.graph[i].state.base_tf.translation;
						distance_edges.emplace_back(origin, dest);
					}
				}
				distance_edges_viz.updateLine(distance_edges);
			} else if (!unoptimized_path) {
				// Plan a TSP over the PRM.
				auto tour = pick_visitation_order(distance_lookup,
				                                  start_to_goals_distances,
				                                  &*group_index_table,
				                                  group_sizes);

				// Construct the final path based on the TSP solution and predecessor lookup tables.
				RobotPath path = construct_final_path(prm.graph,
				                                      predecessor_lookup,
				                                      start_to_goals_predecessors,
				                                      goal_nodes,
				                                      tour);

				// Visualize the path.
				visualization::visualize_ladder_trace(robot, path, viewer);

				// Register that we solved the problem.
				unoptimized_path = path;
			} else {
				//viewer.stop();
			}
		}

	);

	viewer.start();
}
