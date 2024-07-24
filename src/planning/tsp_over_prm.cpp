// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 7/19/24.
//

#include "tsp_over_prm.h"

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/to_container.hpp>

#include "collision_detection.h"
#include "goal_sampling.h"
#include "state_tools.h"
#include "traveling_salesman.h"
#include "nearest_neighbours/GreedyKCenters.h"

namespace mgodpl {
	/**
	 * @brief Add a new node to the PRM, connecting it up to the nearest neighbors.
	 *
	 * @param	state						The state of the new node.
	 * @param	prm							The PRM to add the node to.
	 * @param	k_neighbors					The number of neighbors to connect to.
	 * @param   goal_index					The index of the goal this node represents, if it is a goal node.
	 * @param   check_motion_collides		A function that checks if a motion between two states collides.
	 * @param   hooks						Optional hooks to call at various points in the process.
	 *
	 * @returns The vertex descriptor of the new node.
	 */
	PRMGraph::vertex_descriptor add_and_connect_roadmap_node(
		const RobotState &state,
		PRMGraph &prm,
		const PRMGraphSpatialIndex &spatial_index,
		size_t k_neighbors,
		std::optional<std::pair<size_t, size_t> > goal_index,
		const std::function<bool(
			const RobotState &,
			const RobotState &)> &check_motion_collides,
		const std::optional<AddRoadmapNodeHooks> &hooks
	) {
		std::vector<std::pair<RobotState, PRMGraph::vertex_descriptor> > k_nearest;
		spatial_index.nearestK({state, 0}, k_neighbors, k_nearest);

		// Add the new node to the graph. (Note: we do this AFTER finding the neighbors, so we don't connect to ourselves.)
		auto new_vertex = boost::add_vertex({state, goal_index}, prm);

		// Then add the edges:
		for (const auto &[neighbor_state, neighbor]: k_nearest) {
			// Check if the motion collides.
			bool collides = check_motion_collides(neighbor_state, state);

			// Call the hooks.
			if (hooks) hooks->on_edge_considered({state, new_vertex}, {neighbor_state, neighbor}, !collides);

			if (!collides) {
				// Add an edge to the graph if it doesn't collide.
				boost::add_edge(new_vertex, neighbor, equal_weights_distance(state, neighbor_state), prm);
			}
		}

		// Return the graph vertex id.
		return new_vertex;
	}

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
	std::pair<std::vector<double>, std::vector<PRMGraph::vertex_descriptor> > runDijkstra(
		const PRMGraph &graph,
		PRMGraph::vertex_descriptor start_node
	) {
		std::vector<PRMGraph::vertex_descriptor> predecessors(num_vertices(graph));
		std::vector<double> distances(num_vertices(graph));

		dijkstra_shortest_paths(graph,
		                        start_node,
		                        predecessor_map(
			                        make_iterator_property_map(predecessors.begin(),
			                                                   get(boost::vertex_index, graph)))
		                        .distance_map(
			                        make_iterator_property_map(distances.begin(),
			                                                   get(boost::vertex_index, graph))));

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
	RobotPath retrace_path(
		const PRMGraph &graph,
		const std::vector<PRMGraph::vertex_descriptor> &predecessor_lookup,
		PRMGraph::vertex_descriptor goal_node
	) {
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
	std::vector<size_t> convert_tour_to_global(
		const GroupIndexTable &group_index_table,
		const std::vector<std::pair<size_t, size_t> > &tour
	) {
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
	RobotPath construct_final_path(
		const PRMGraph &graph,
		const std::vector<std::vector<PRMGraph::vertex_descriptor> > &predecessor_lookup,
		const std::vector<PRMGraph::vertex_descriptor> &start_to_goals_predecessors,
		const std::vector<PRMGraph::vertex_descriptor> &goal_nodes,
		const std::vector<size_t> &tour
	) {
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

	/**
	 * Take a sample and connect it to the roadmap if it doesn't collide.
	 *
	 * @param prm						The PRM to add the node to.
	 * @param k_neighbors				The number of neighbors to connect to.
	 * @param sample_state_at_random	A function that generates a random state.
	 * @param check_state_collides		A function that checks if a state collides with the environment.
	 * @param check_motion_collides		A function that checks if a motion between two states collides.
	 * @param hooks						Optional hooks for observing the process.
	 * @return True if the sample was added to the roadmap, false if it was discarded.
	 */
	bool sample_and_connect_infrastucture_node(
		PRMGraph &prm,
		PRMGraphSpatialIndex &spatial_index,
		size_t k_neighbors,
		std::function<RobotState()> &sample_state_at_random,
		const std::function<bool(const RobotState &)> &check_state_collides,
		const std::function<bool(const RobotState &, const RobotState &)> &
		check_motion_collides,
		const std::optional<InfrastructureSampleHooks> &hooks
	) {
		// Generate a random state.
		auto state = sample_state_at_random();

		// Check if the robot collides with the tree.
		bool collides = check_state_collides(state);

		if (hooks) hooks->on_sample(state, !collides);

		// If this sample collides, don't add it to the roadmap.
		if (collides) {
			return false;
		}

		// Otherwise, add it to the roadmap, and try to connect it to the nearest neighbors.
		auto new_vertex = add_and_connect_roadmap_node(state,
		                                               prm,
		                                               spatial_index,
		                                               k_neighbors,
		                                               std::nullopt,
		                                               check_motion_collides,
		                                               hooks ? hooks->add_roadmap_node_hooks : std::nullopt);

		// If it's not a goal sample, add it to the infrastructure nodes.
		spatial_index.add({state, new_vertex});


		return true;
	}

	/**
	 * Take a goal sample and connect it to the roadmap if it doesn't collide.
	 *
	 * @param prm						The PRM to add the node to.
	 * @param spatial_index				The spatial index to use for nearest neighbor queries.
	 * @param params					The parameters for sampling and connecting goal nodes.
	 * @param goal_group_id				The index of the goal group.
	 * @param sample_goal_state			A function that generates a random state from the goal region.
	 * @param check_state_collides		A function that checks if a state collides with the environment.
	 * @param check_motion_collides		A function that checks if a motion between two states collides.
	 * @param hooks						Optional hooks for observing the process.
	 *
	 * @return The vertex IDs of the goal samples that were added to the roadmap.
	 */
	std::vector<PRMGraph::vertex_descriptor> sample_and_connect_goal_states(
		PRMGraph &prm,
		const PRMGraphSpatialIndex &spatial_index,
		const GoalSampleParams &params,
		size_t goal_group_id,
		std::function<RobotState()> &sample_goal_state,
		const std::function<bool(const RobotState &)> &
		check_state_collides,
		const std::function<bool(const RobotState &, const RobotState &)> &
		check_motion_collides,
		const std::optional<GoalSampleHooks> &
		hooks
	) {
		// Reserve space for the valid samples.
		std::vector<PRMGraph::vertex_descriptor> valid_samples;
		valid_samples.reserve(params.max_valid_samples);

		// Keep iterating until we have enough valid samples or we've tried too many times.
		for (size_t attempt = 0; attempt < params.max_attempts && valid_samples.size() < params.max_valid_samples; ++
		     attempt) {
			// Sample a goal state.
			auto goal_state = sample_goal_state();

			// Check if the robot collides with the tree.
			bool collides = check_state_collides(goal_state);

			// Call the hooks, if they exist.
			if (hooks) hooks->on_sample(goal_state, !collides);

			// If it doesn't collide, add it to the roadmap.
			if (!collides) {
				// Add the goal state to the roadmap.
				auto new_vertex = add_and_connect_roadmap_node(goal_state,
				                                               prm,
				                                               spatial_index,
				                                               params.k_neighbors,
				                                               std::make_pair(goal_group_id, valid_samples.size()),
				                                               check_motion_collides,
				                                               hooks ? hooks->add_roadmap_node_hooks : std::nullopt);

				// Add it to the list of valid samples.
				valid_samples.push_back(new_vertex);
			}
		}

		// Return the valid samples.
		return valid_samples;
	}

	std::vector<double> filter_goal_distances_vector(
		const std::vector<PRMGraph::vertex_descriptor> &goal_nodes,
		const std::vector<double> &distances
	) {
		return goal_nodes
		       | ranges::views::transform([&](auto goal_node) { return distances[goal_node]; })
		       | ranges::to<std::vector<double> >();
	}

	/// A combination of a RobotState and a PRMGraph vertex descriptor, used as entries into the GNAT spatial index.
	/// The RobotState should be identical to the one stored in the PRMGraph.
	/// (TODO: This creates potentially quite some redundancy, as the state is stored at least twice.)
	using GNATPoint = std::pair<RobotState, PRMGraph::vertex_descriptor>;

	/**
	 * Initialize an empty GNAT spatial index over the robot states and graph vertex descriptors.
	 *
	 * @param rng	The random number generator to use for the GNAT index (must outlive the index)
	 * @return	The initialized GNAT index
	 */
	ompl::NearestNeighborsGNAT<GNATPoint> init_empty_spatial_index(
		random_numbers::RandomNumberGenerator &rng
	) {
		// The distance function for the GNAT index, which extracts the RobotState from the GNATPoint and measures the distance between them.
		// TODO: don't hardcode equal_weights_distance here.
		std::function gnat_dist_fn = [&](const GNATPoint &a, const GNATPoint &b) {
			return equal_weights_distance(a.first, b.first);
		};

		// The pivot selection function for the GNAT index, which uses the greedy k-centers algorithm to select pivots.
		// Note that it takes a reference to the rng, as greedy_k_centers requires a random number generator.
		// NearestNeighborsGNAT itself is not aware of the RNG.
		ompl::NearestNeighborsGNAT<GNATPoint>::SelectPivotFn select_pivots = [gnat_dist_fn, &rng](
			const std::vector<GNATPoint> &data,
			unsigned int k) {
			return greedy_k_centers<GNATPoint>(data,
			                                   k,
			                                   gnat_dist_fn,
			                                   rng,
			                                   std::nullopt);
		};

		// Initialize the GNAT index.
		ompl::NearestNeighborsGNAT gnat(select_pivots);

		// Set the distance function. (TODO: make this a constructor argument)
		gnat.setDistanceFunction(
			[](const auto &a, const auto &b) {
				return equal_weights_distance(a.first, b.first);
			});

		// Return the initialized GNAT index.
		return gnat;
	}

	/**
	 * Build the infrastructure roadmap for the TSP over PRM algorithm.
	 *
	 * This algorithm repeatedly:
	 * - Samples a random state.
	 * - Checks if it does not collide with the tree.
	 * - If so, adds it to the roadmap and connects it to the nearest neighbors.
	 *
	 * This repeats for a number of iterations, as specified in `parameters.max_samples`.
	 *
	 * @param prm				The PRM to build the infrastructure roadmap on
	 * @param spatial_index		The spatial index to use for nearest neighbor queries; new nodes are added to this index
	 * @param parameters		The parameters for the TSP over PRM algorithm
	 * @param sample_uniform	A function to sample a random state
	 * @param state_collides	A function to check if a state collides with the tree
	 * @param motion_collides	A function to check if a motion between two states collides with the tree
	 * @param hooks				Optional hooks to observe the behavior of the algorithm
	 */
	void build_infrastructure_roadmap(PRMGraph &prm,
	                                  PRMGraphSpatialIndex &spatial_index,
	                                  const TspOverPrmParameters &parameters,
	                                  std::function<RobotState()> sample_uniform,
	                                  std::function<bool(const RobotState &)> state_collides,
	                                  std::function<bool(const RobotState &, const RobotState &)> motion_collides,
	                                  const std::optional<TspOverPrmHooks> &hooks) {
		// Sample infrastructure nodes.
		for (size_t i = 0; i < parameters.max_samples; ++i) {
			sample_and_connect_infrastucture_node(prm,
			                                      spatial_index,
			                                      parameters.n_neighbours,
			                                      sample_uniform,
			                                      state_collides,
			                                      motion_collides,
			                                      hooks
				                                      ? std::make_optional(hooks->infrastructure_sample_hooks)
				                                      : std::nullopt);
		}
	}

	/**
	 * Sample goal states for the TSP over PRM algorithm, and connect them to the roadmap.
	 *
	 * @param fruit_positions		The positions of the fruits to sample goal states for.
	 * @param prm					The PRM to connect the goal states to.
	 * @param spatial_index			The spatial index to use for nearest neighbor queries. Goal states are not added to this index.
	 * @param parameters			The parameters for the TSP over PRM algorithm.
	 * @param sample_goal_state		A function to sample a goal state for a given fruit index. (Does not check for collisions.)
	 * @param state_collides		A function to check if a state collides with the tree.
	 * @param motion_collides		A function to check if a motion between two states collides with the tree.
	 * @param rng					The random number generator to use for sampling.
	 * @param hooks					Optional hooks to observe the behavior of the algorithm.
	 * @return A pair of vectors: the goal nodes in the PRM, and the number of goal samples for each fruit.
	 */
	std::pair<std::vector<PRMGraph::vertex_descriptor>, std::vector<size_t> > sample_goal_states(
		const std::vector<math::Vec3d> &fruit_positions,
		PRMGraph &prm,
		const PRMGraphSpatialIndex &spatial_index,
		const TspOverPrmParameters &parameters,
		const std::function<RobotState(size_t)> &sample_goal_state,
		const std::function<bool(const RobotState &)> &state_collides,
		const std::function<bool(const RobotState &, const RobotState &)> &motion_collides,
		random_numbers::RandomNumberGenerator &rng,
		const std::optional<TspOverPrmHooks> &hooks
	) {
		// Store the goal sample vertex nodes, with a separate sub-vector for each goal.
		// Start with a vector of empty vectors.
		std::vector<PRMGraph::vertex_descriptor> goal_nodes;

		std::vector<size_t> group_sizes(fruit_positions.size(), 0);

		// Sample goal nodes.
		for (size_t goal_index = 0; goal_index < fruit_positions.size(); ++goal_index) {
			std::function goal_sample = [&]() { return sample_goal_state(goal_index); };

			auto goal_sample_states = sample_and_connect_goal_states(
				prm,
				spatial_index,
				parameters.goal_sample_params,
				goal_index,
				goal_sample,
				state_collides,
				motion_collides,
				hooks ? std::make_optional(hooks->goal_sample_hooks) : std::nullopt);

			// Store the goal nodes.
			goal_nodes.insert(goal_nodes.end(), goal_sample_states.begin(), goal_sample_states.end());

			// Store the number of goal samples for this fruit.
			group_sizes[goal_index] = goal_sample_states.size();
		}

		return {goal_nodes, group_sizes};
	}

	/**
	 * This structure contains the distance lookup tables and predecessor lookup tables
	 * necessary to reconstruct the paths between goals and from the start to the goals.
	 */
	struct GoalToGoalPathResults {
		/// Distance lookup tables between goal samples.
		std::vector<std::vector<double> > distance_lookup; ///< Distance lookup table between goal samples.
		/// Distance lookup table from the start node to each goal sample.
		std::vector<double> start_to_goals_distances; ///< Distances from the start node to each goal sample.

		/// Predecessor lookup tables for reconstructing goal-to-goal paths.
		std::vector<std::vector<PRMGraph::vertex_descriptor> > predecessor_lookup;
		/// Predecessor lookup table for reconstructing paths from the start to each goal.
		std::vector<PRMGraph::vertex_descriptor> start_to_goals_predecessors;
	};

	/**
	 * @brief Calculate the distances and paths between goal nodes and from the start node to goal nodes.
	 *
	 * This function runs Dijkstra's algorithm from each goal node to calculate the distances and predecessor
	 * lookup tables for reconstructing the paths. It also runs Dijkstra's algorithm from the start node to
	 * calculate the distances and predecessor lookup table for paths from the start to each goal node.
	 *
	 * @param graph The PRM graph.
	 * @param goal_nodes The goal nodes in the PRM graph.
	 * @param group_index_table The group index table for goal nodes.
	 * @return A GoalToGoalPathResults structure containing the distance and predecessor lookup tables.
	 */
	GoalToGoalPathResults calculate_goal_to_goal_paths(
		const PRMGraph &graph,
		const std::vector<PRMGraph::vertex_descriptor> &goal_nodes,
		const GroupIndexTable &group_index_table
	) {
		GoalToGoalPathResults results;
		results.distance_lookup.resize(group_index_table.total());
		results.predecessor_lookup.resize(group_index_table.total());

		// Calculate the distances between all goal samples.
		for (size_t i = 0; i < group_index_table.total(); ++i) {
			// Run Dijkstra's algorithm from the goal node.
			std::tie(results.distance_lookup[i], results.predecessor_lookup[i]) = runDijkstra(graph, goal_nodes[i]);
		}

		// Calculate the distances from the start to all goal samples.
		// Run Dijkstra's algorithm from the start node.
		std::tie(results.start_to_goals_distances, results.start_to_goals_predecessors) = runDijkstra(graph, 0);

		return results;
	}

	RobotPath plan_path_tsp_over_prm(
		const RobotState &start_state,
		const std::vector<math::Vec3d> &fruit_positions,
		const robot_model::RobotModel &robot,
		const fcl::CollisionObjectd &tree_collision,
		const TspOverPrmParameters &parameters,
		random_numbers::RandomNumberGenerator &rng,
		const std::optional<TspOverPrmHooks> &hooks
	) {
		// Look up the link IDs for the base and end effector. (TODO: could potentially abstract this away into a goal sampler function?)
		robot_model::RobotModel::LinkId base_link = robot.findLinkByName("flying_base");
		robot_model::RobotModel::LinkId end_effector_link = robot.findLinkByName("end_effector");

		// Allocate an empty prm.
		PRMGraph prm;
		PRMGraphSpatialIndex infrastructure_spatial_index = init_empty_spatial_index(rng);

		// We allocate a few functions here to hide details regarding collision checking and sampling from the algorithm.

		// Uniform sampling function:
		std::function sample_uniform = [&]() {
			return generateUniformRandomState(robot, rng, 5.0, 10.0);
		};

		// Collision check function:
		std::function state_collides = [&](const RobotState &state) {
			return check_robot_collision(robot, tree_collision, state);
		};

		// Motion collision check function:
		std::function motion_collides = [&](const RobotState &a, const RobotState &b) {
			return check_motion_collides(robot, tree_collision, a, b);
		};

		// Goal sampling function for a given goal index:
		std::function sample_goal_state = [&](size_t goal_index) {
			return genGoalStateUniform(
				rng,
				fruit_positions[goal_index],
				robot,
				base_link,
				end_effector_link
			);
		};

		// Add the start state to the roadmap.
		add_and_connect_roadmap_node(start_state,
		                             prm,
		                             infrastructure_spatial_index,
		                             parameters.n_neighbours,
		                             std::nullopt,
		                             motion_collides,
		                             std::nullopt);

		// Build the infrastructure roadmap.
		build_infrastructure_roadmap(prm,
		                             infrastructure_spatial_index,
		                             parameters,
		                             sample_uniform,
		                             state_collides,
		                             motion_collides,
		                             hooks);

		// Sample goal states and connect them to the roadmap.
		const auto &[goal_nodes, group_sizes] =
				sample_goal_states(fruit_positions,
				                   prm,
				                   infrastructure_spatial_index,
				                   parameters,
				                   sample_goal_state,
				                   state_collides,
				                   motion_collides,
				                   rng,
				                   hooks);

		// Create a group index table.
		GroupIndexTable group_index_table(group_sizes);

		const auto &goal_to_goal_paths = calculate_goal_to_goal_paths(
			prm,
			goal_nodes,
			group_index_table);

		// Plan a visitation order based on the distance lookup tables.
		auto visitation_order = pick_visitation_order(
			goal_to_goal_paths.distance_lookup,
			goal_to_goal_paths.start_to_goals_distances,
			&group_index_table,
			group_sizes
		);

		// Construct the final path based on the TSP solution and predecessor lookup tables.
		auto final_path = construct_final_path(
			prm,
			goal_to_goal_paths.predecessor_lookup,
			goal_to_goal_paths.start_to_goals_predecessors,
			goal_nodes,
			visitation_order
		);

		// Return the final path.
		return final_path;
	}
} // mgodpl
