// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 7/19/24.
//

#ifndef TSP_OVER_PRM_H
#define TSP_OVER_PRM_H
#include <optional>
#include <boost/graph/adjacency_list.hpp>
#include <fcl/narrowphase/collision_object.h>

#include "GroupIndexTable.h"
#include "RandomNumberGenerator.h"
#include "RobotModel.h"
#include "RobotPath.h"
#include "RobotState.h"

namespace mgodpl {
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
	using PRMGraph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperties,
		boost::property<boost::edge_weight_t, double> >;

	/**
	 * A container struct for a two-tier multi-goal PRM.
	 *
	 * It maintains a list of "infrastructure nodes", which are considered the candidates for nearest-neighbour
	 * connections; this implies that anything *not* on that list is considered a goal node and will not be
	 * connected to.
	 */
	struct TwoTierMultigoalPRM {
		// The Boost Graph representing the roadmap.
		PRMGraph graph;

		// A vector of all the infrastructure nodes. (TODO: make this a spatial index instead.)
		std::vector<PRMGraph::vertex_descriptor> infrastructure_nodes;
	};


	/**
	 * This function looks up the k nearest neighbors to a given state. These are infrastructure nodes only.
	 *
	 * @param state		The state to find the neighbors for; it need not yet be in the graph.
	 * @param k			The number of neighbors to find.
	 * @return			A vector of the k nearest neighbors, paired with their distances. (This may be less than k if there are fewer nodes in the graph.)
	 */
	std::vector<std::pair<double, PRMGraph::vertex_descriptor> > k_nearest_neighbors(
		const TwoTierMultigoalPRM &prm,
		const RobotState &state,
		size_t k);

	struct AddRoadmapNodeHooks {
		/// A function called when an edge is considered, with the source and target states, and a boolean decision (true if added, false if not).
		std::function<void(
			std::pair<const RobotState &, const PRMGraph::vertex_descriptor &>,
			std::pair<const RobotState &, const PRMGraph::vertex_descriptor &>,
			bool)>
		on_edge_considered;
	};

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
	PRMGraph::vertex_descriptor add_and_connect_roadmap_node(const RobotState &state,
	                                                         TwoTierMultigoalPRM &prm,
	                                                         size_t k_neighbors,
	                                                         std::optional<std::pair<size_t, size_t> > goal_index,
	                                                         const std::function<bool(const RobotState &,
		                                                         const RobotState &)> &
	                                                         check_motion_collides,
	                                                         const std::optional<AddRoadmapNodeHooks> &hooks =
			                                                         std::nullopt
	);

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
	std::pair<std::vector<double>, std::vector<PRMGraph::vertex_descriptor> >
	runDijkstra(const PRMGraph &graph,
	            PRMGraph::vertex_descriptor start_node);

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
	[[nodiscard]] RobotPath retrace_path(const PRMGraph &graph,
	                                     const std::vector<PRMGraph::vertex_descriptor> &
	                                     predecessor_lookup,
	                                     PRMGraph::vertex_descriptor goal_node);

	/**
	 * Convert a TSP solution referring to fruit-and-sample indices to one using only global goal sample indices.
	 *
	 * @param group_index_table	The group index table, to translate the index pairs from the TSP solution to global goal sample
	 * @param tour				The TSP solution.
	 *
	 * @returns			The TSP solution with global goal sample indices.
	 */
	std::vector<size_t> convert_tour_to_global(const GroupIndexTable &group_index_table,
	                                           const std::vector<std::pair<size_t, size_t> > &tour);

	/**
	 * @brief Plan a visitation order based on the distance lookup tables. This is an abstraction over tsp_open_end_grouped that avoids lambdas.
	 * @return The visitation order, expressed in terms of global goal sample indices.
	 */
	std::vector<size_t> pick_visitation_order(
		const std::vector<std::vector<double> > &distance_lookup,
		const std::vector<double> &start_to_goals_distances,
		const GroupIndexTable *group_index_table,
		const std::vector<size_t> &group_sizes
	);

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
	RobotPath construct_final_path(const PRMGraph &graph,
	                               const std::vector<std::vector<PRMGraph::vertex_descriptor> > &
	                               predecessor_lookup,
	                               const std::vector<PRMGraph::vertex_descriptor> &
	                               start_to_goals_predecessors,
	                               const std::vector<PRMGraph::vertex_descriptor> &goal_nodes,
	                               const std::vector<size_t> &tour);

	/**
	 * A set of hooks for sampling and connecting infrastructure nodes.
	 */
	struct InfrastructureSampleHooks {
		// A callback for when a sample is taken, with the state and a boolean indicating whether it was added.
		std::function<void(const RobotState &state, bool added)> on_sample;
		// Sub-hooks for connecting the roadmap node to the nearest neighbors.
		std::optional<AddRoadmapNodeHooks> add_roadmap_node_hooks;
	};

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
		TwoTierMultigoalPRM &prm,
		size_t k_neighbors,
		std::function<RobotState()> &sample_state_at_random,
		const std::function<bool(const RobotState &)> &check_state_collides,
		const std::function<bool(const RobotState &, const RobotState &)> &check_motion_collides,
		const std::optional<InfrastructureSampleHooks> &hooks
	);

	/**
	 * A set of hooks for sampling and connecting goal nodes.
	 */
	struct GoalSampleHooks {
		/// A callback for when a sample is taken, with the state and a boolean indicating whether it was added.
		std::function<void(const RobotState &sampled, bool added)> on_sample;
		// Sub-hooks for connecting the roadmap node to the nearest neighbors.
		std::optional<AddRoadmapNodeHooks> add_roadmap_node_hooks;
	};

	/**
	 * A small struct of parameters for sampling and connecting goal nodes.
	 */
	struct GoalSampleParams {
		/// The number of (infrastructure) neighbors to connect to.
		size_t k_neighbors = 5;
		/// The maximum number of valid samples to take.
		size_t max_valid_samples = 2;
		/// The maximum number of attempts to take a valid sample. (Total; not affected by max_valid_samples.)
		size_t max_attempts = 100;
	};

	/**
	 * Take a goal sample and connect it to the roadmap if it doesn't collide.
	 *
	 * @param prm						The PRM to add the node to.
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
		TwoTierMultigoalPRM &prm,
		const GoalSampleParams &params,
		size_t goal_group_id,
		std::function<RobotState()> &sample_goal_state,
		const std::function<bool(const RobotState &)> &check_state_collides,
		const std::function<bool(const RobotState &, const RobotState &)> &check_motion_collides,
		const std::optional<GoalSampleHooks> &hooks
	);


	/**
	 * @brief Filters the distances towards goal nodes from the distances vector.
	 *
	 * @param goal_nodes	A vector of vertex descriptors representing the goal nodes in the PRM graph.
	 * @param distances		A vector containing the distances between nodes in the PRM graph, indexed by the vertex descriptors.
	 *
	 * @return A vector of distances that correspond to the goal nodes.
	 */
	std::vector<double> filter_goal_distances_vector(const std::vector<PRMGraph::vertex_descriptor> &goal_nodes,
	                                                 const std::vector<double> &distances);

	struct TspOverPrmParameters {
		/// The number of nearest neighbors to connect to.
		size_t n_neighbours = 5;
		/// The number of samples to take.
		size_t max_samples = 100;
		/// The number of samples to take per goal.
		GoalSampleParams goal_sample_params;
	};

	struct TspOverPrmHooks {
		/// Hooks for sampling and connecting infrastructure nodes.
		InfrastructureSampleHooks infrastructure_sample_hooks;
		/// Hooks for sampling and connecting goal nodes.
		GoalSampleHooks goal_sample_hooks;
	};

	/**
	 * @brief Plan a path around a fruit tree using the TSP-over-PRM method.
	 *
	 * This function implements a variant of the Traveling Salesman Problem (TSP)
	 * over a Probabilistic Roadmap (PRM) to plan a path for a robot to visit all
	 * specified fruit positions around a tree.
	 *
	 * @param start_state The initial state of the robot.
	 * @param fruit_positions A vector of positions representing the locations of fruits to be collected.
	 * @param robot The robot model, containing information about the robot's kinematics and dynamics.
	 * @param tree_collision The collision object representing the tree trunk, to check for collisions between the robot and the tree.
	 * @param parameters A struct containing parameters for the TSP-over-PRM algorithm, such as the number of samples to take and the number of nearest neighbors to connect.
	 * @param rng A random number generator used throughout the planning process.
	 * @param hooks (Optional) An object containing various hooks for observing the progress of the algorithm.
	 *
	 * @return RobotPath The planned path, including all the states the robot should go through to visit all fruits.
	 */
	RobotPath plan_path_tsp_over_prm(
		const RobotState &start_state,
		const std::vector<math::Vec3d> &fruit_positions,
		const robot_model::RobotModel &robot,
		const fcl::CollisionObjectd &tree_collision,
		const TspOverPrmParameters &parameters,
		random_numbers::RandomNumberGenerator &rng,
		const std::optional<TspOverPrmHooks> &hooks = std::nullopt
	);
} // mgodpl

#endif //TSP_OVER_PRM_H
