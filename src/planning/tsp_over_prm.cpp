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
	PRMGraph::vertex_descriptor add_and_connect_roadmap_node(
		const RobotState &state,
		TwoTierMultigoalPRM &prm,
		size_t k_neighbors,
		std::optional<std::pair<size_t, size_t> > goal_index,
		const std::function<bool(
			const RobotState &,
			const RobotState &)> &check_motion_collides,
		const std::optional<AddRoadmapNodeHooks> &hooks
	) {
		std::vector<std::pair<RobotState, PRMGraph::vertex_descriptor> > k_nearest;
		prm.infrastructure_nodes.nearestK({state, 0}, k_neighbors, k_nearest);

		// Add the new node to the graph. (Note: we do this AFTER finding the neighbors, so we don't connect to ourselves.)
		auto new_vertex = boost::add_vertex({state, goal_index}, prm.graph);

		// Then add the edges:
		for (const auto &[neighbor_state, neighbor]: k_nearest) {
			// Check if the motion collides.
			bool collides = check_motion_collides(neighbor_state, state);

			// Call the hooks.
			if (hooks) hooks->on_edge_considered({state, new_vertex}, {neighbor_state, neighbor}, !collides);

			if (!collides) {
				// Add an edge to the graph if it doesn't collide.
				boost::add_edge(new_vertex, neighbor, equal_weights_distance(state, neighbor_state), prm.graph);
			}
		}

		// If it's not a goal sample, add it to the infrastructure nodes.
		if (!goal_index) {
			prm.infrastructure_nodes.add({state, new_vertex});
		}

		// Return the graph vertex id.
		return new_vertex;
	}

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

	std::vector<size_t> convert_tour_to_global(
		const GroupIndexTable &group_index_table,
		const std::vector<std::pair<size_t, size_t> > &tour
	) {
		return tour | ranges::views::transform([&](const auto &pair) {
			return group_index_table.lookup(pair.first, pair.second);
		}) | ranges::to<std::vector>();
	}

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

	bool sample_and_connect_infrastucture_node(
		TwoTierMultigoalPRM &prm,
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
		add_and_connect_roadmap_node(state,
		                             prm,
		                             k_neighbors,
		                             std::nullopt,
		                             check_motion_collides,
		                             hooks ? hooks->add_roadmap_node_hooks : std::nullopt);

		return true;
	}

	std::vector<PRMGraph::vertex_descriptor> sample_and_connect_goal_states(
		TwoTierMultigoalPRM &prm,
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
		for (int attempt = 0; attempt < params.max_attempts && valid_samples.size() < params.max_valid_samples; ++
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

	RobotPath plan_path_tsp_over_prm(
		const RobotState &start_state,
		const std::vector<math::Vec3d> &fruit_positions,
		const robot_model::RobotModel &robot,
		const fcl::CollisionObjectd &tree_collision,
		const TspOverPrmParameters &parameters,
		random_numbers::RandomNumberGenerator &rng,
		const std::optional<TspOverPrmHooks> &hooks
	) {
		using GNATPoint = std::pair<RobotState, PRMGraph::vertex_descriptor>;

		std::function gnat_dist_fn = [&](const GNATPoint &a, const GNATPoint &b) {
			return equal_weights_distance(a.first, b.first);
		};

		ompl::NearestNeighborsGNAT<GNATPoint>::SelectPivotFn select_pivots = [&](
			const std::vector<GNATPoint> &data,
			unsigned int k) {
			return greedy_k_centers(data,
			                        k,
			                        gnat_dist_fn,
			                        rng,
			                        std::nullopt);
		};

		// Allocate an empty prm.
		TwoTierMultigoalPRM prm{
			.graph = {},
			.infrastructure_nodes = ompl::NearestNeighborsGNAT(select_pivots)
		};
		prm.infrastructure_nodes.setDistanceFunction(
			[&](const auto &a, const auto &b) {
				return equal_weights_distance(a.first, b.first);
			});

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

		// Add the start state to the roadmap.
		add_and_connect_roadmap_node(start_state, prm, parameters.n_neighbours, std::nullopt, motion_collides);

		// Store the goal sample vertex nodes, with a separate sub-vector for each goal.
		// Start with a vector of empty vectors.
		std::vector<PRMGraph::vertex_descriptor> goal_nodes;

		// Look up the link IDs for the base and end effector.
		robot_model::RobotModel::LinkId base_link = robot.findLinkByName("flying_base");
		robot_model::RobotModel::LinkId end_effector_link = robot.findLinkByName("end_effector");

		std::vector<size_t> group_sizes(fruit_positions.size(), 0);

		// Sample infrastructure nodes.
		for (size_t i = 0; i < parameters.max_samples; ++i) {
			sample_and_connect_infrastucture_node(prm,
			                                      parameters.n_neighbours,
			                                      sample_uniform,
			                                      state_collides,
			                                      motion_collides,
			                                      hooks
				                                      ? std::make_optional(hooks->infrastructure_sample_hooks)
				                                      : std::nullopt);
		}

		// Sample goal nodes.
		for (size_t goal_index = 0; goal_index < fruit_positions.size(); ++goal_index) {
			std::function goal_sample = [&]() {
				return genGoalStateUniform(
					rng,
					fruit_positions[goal_index],
					robot,
					base_link,
					end_effector_link
				);
			};

			auto goal_sample_states = sample_and_connect_goal_states(
				prm,
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

		// Create a group index table.
		GroupIndexTable group_index_table(group_sizes);

		// Allocate the distance lookup table.
		// Store a goal-sample-to-goal-sample distance lookup table.
		// Given two goal samples indices i and j, distance_lookup[i][j] contains
		// the distance from goal sample i to goal sample j through the PRM.
		//
		// Note: these are global goal sample indices, not graph vertex IDs. (See group_index_table)
		std::vector<std::vector<double> > distance_lookup;
		std::vector<double> start_to_goals_distances;

		// And a predecessor lookup table to reconstruct the paths.
		std::vector<std::vector<PRMGraph::vertex_descriptor> > predecessor_lookup;
		std::vector<PRMGraph::vertex_descriptor> start_to_goals_predecessors;

		distance_lookup.resize(group_index_table.total());
		predecessor_lookup.resize(group_index_table.total());

		// Calculate the distances between all goal samples.
		for (size_t i = 0; i < group_index_table.total(); ++i) {
			// Run Dijkstra's algorithm from the goal node.
			auto [distances, predecessors] = runDijkstra(prm.graph, goal_nodes[i]);

			// Store the distances and predecessors.
			distance_lookup[i] = distances;
			predecessor_lookup[i] = predecessors;
		}

		// Calculate the distances from the start to all goal samples.
		{
			// Run Dijkstra's algorithm from the start node.
			auto [distances, predecessors] = runDijkstra(prm.graph, 0);

			// Store the distances and predecessors.
			start_to_goals_distances = filter_goal_distances_vector(goal_nodes, distances);
			start_to_goals_predecessors = predecessors;
		}

		// Plan a visitation order based on the distance lookup tables.
		auto visitation_order = pick_visitation_order(
			distance_lookup,
			start_to_goals_distances,
			&group_index_table,
			group_sizes
		);

		// Construct the final path based on the TSP solution and predecessor lookup tables.
		auto final_path = construct_final_path(
			prm.graph,
			predecessor_lookup,
			start_to_goals_predecessors,
			goal_nodes,
			visitation_order
		);

		// Return the final path.
		return final_path;
	}
} // mgodpl
