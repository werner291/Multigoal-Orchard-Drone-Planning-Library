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
using PRMGraph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperties, boost::property<
	boost::edge_weight_t, double> >;

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
	size_t k) {
	// Keep the distances and the indices.
	std::vector<std::pair<double, PRMGraph::vertex_descriptor> > distances;

	// Reserve k+1 elements, so we can insert the new element at the correct position.
	distances.reserve(k + 1);

	// Iterate over all infrastructure nodes (TODO: do better than this linear search.)
	for (auto v: prm.infrastructure_nodes) {
		// Get the properties of the node (i.e., the state).
		const auto &node = prm.graph[v];

		bool inserted = false;

		// Iterate over all neighbors found so far.
		for (size_t i = 0; i < distances.size(); ++i) {
			// If the distance is smaller than the current distance, insert it.
			if (distances[i].first > equal_weights_distance(node.state, state)) {
				distances.insert(distances.begin() + static_cast<long>(i),
				                 {equal_weights_distance(node.state, state), v});

				// Mark that we've inserted the element.
				inserted = true;

				// If it's now larger than n_neighbours, remove the last element.
				if (distances.size() > k) {
					distances.pop_back();
				}
				break; // Once we've inserted, we're done.
			}
		}

		// If we haven't inserted it yet, and there's still space, add it to the end.
		if (!inserted && distances.size() < k) {
			distances.emplace_back(equal_weights_distance(node.state, state), v);
		}
	}

	return distances;
}

struct AddRoadmapNodeHooks {
	/// A function called when an edge is considered, with the source and target states, and a boolean decision (true if added, false if not).
	std::function<void(const PRMGraph::vertex_descriptor &, const PRMGraph::vertex_descriptor &, bool)>
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
                                                         const std::function<bool(
	                                                         const RobotState &,
	                                                         const RobotState &)> &
                                                         check_motion_collides,
                                                         const std::optional<AddRoadmapNodeHooks> &hooks = std::nullopt
) {
	// Find the k nearest neighbors. (Brute-force; should migrate to a VP-tree when I can.)
	auto k_nearest = k_nearest_neighbors(prm, state, k_neighbors);

	// Add the new node to the graph. (Note: we do this AFTER finding the neighbors, so we don't connect to ourselves.)
	auto new_vertex = boost::add_vertex({state, goal_index}, prm.graph);

	// Then add the edges:
	for (const auto &[distance, neighbor]: k_nearest) {
		// Check if the motion collides.
		bool collides = check_motion_collides(prm.graph[neighbor].state, state);

		// Call the hooks.
		if (hooks) hooks->on_edge_considered(neighbor, new_vertex, !collides);

		if (!collides) {
			// Add an edge to the graph if it doesn't collide.
			boost::add_edge(new_vertex, neighbor, distance, prm.graph);
		}
	}

	// If it's not a goal sample, add it to the infrastructure nodes.
	if (!goal_index) {
		prm.infrastructure_nodes.push_back(new_vertex);
	}

	// Return the graph vertex id.
	return new_vertex;
}

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
std::pair<std::vector<double>, std::vector<PRMGraph::vertex_descriptor> >
runDijkstra(const PRMGraph &graph,
            PRMGraph::vertex_descriptor start_node) {
	std::vector<PRMGraph::vertex_descriptor> predecessors(num_vertices(graph));
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
[[nodiscard]] RobotPath retrace_path(const PRMGraph &graph,
                                     const std::vector<PRMGraph::vertex_descriptor> &
                                     predecessor_lookup,
                                     PRMGraph::vertex_descriptor goal_node) {
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
RobotPath construct_final_path(const PRMGraph &graph,
                               const std::vector<std::vector<PRMGraph::vertex_descriptor> > &
                               predecessor_lookup,
                               const std::vector<PRMGraph::vertex_descriptor> &
                               start_to_goals_predecessors,
                               const std::vector<PRMGraph::vertex_descriptor> &goal_nodes,
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

struct TspOverPrmParameters {
	/// The number of nearest neighbors to connect to.
	size_t n_neighbours = 5;
	/// The number of samples to take.
	size_t max_samples = 100;
	/// The number of samples to take per goal.
	size_t max_samples_per_goal = 1;
};

/**
 * A struct with callbacks for observing the progress of the TSP over PRM algorithm.
 *
 * The callbacks in here will be called at various points in the algorithm, allowing the user to visualize the progress.
 *
 * The data relevant to each event is passed as a const reference, allowing it to peek at, but not modify, the internal state.
 */
struct TspOverPrmObserver {
	std::function<void(const RobotState &)> on_sample;
};

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
	size_t k_neighbors;
	/// The maximum number of valid samples to take.
	size_t max_valid_samples;
	/// The maximum number of attempts to take a valid sample. (Total; not affected by max_valid_samples.)
	size_t max_attempts;
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
	const std::optional<InfrastructureSampleHooks> &hooks
) {
	// Reserve space for the valid samples.
	std::vector<PRMGraph::vertex_descriptor> valid_samples;
	valid_samples.reserve(params.max_valid_samples);

	// Keep iterating until we have enough valid samples or we've tried too many times.
	for (int attempt = 0; attempt < params.max_attempts && valid_samples.size() < params.max_valid_samples; ++attempt) {
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

/**
 * @brief Plan a path around a fruit tree using the TSP-over-PRM method.
 *
 * This function implements a variant of the Traveling Salesman Problem (TSP) '
 * over a Probabilistic Roadmap (PRM) to plan a path for a robot to visit all
 * specified fruit positions around a tree.
 *
 * @param initial_state The initial state of the robot.
 * @param fruit_positions A vector of positions representing the locations of fruits to be collected.
 * @param robot_model The robot model, containing information about the robot's kinematics and dynamics.
 * @param tree_trunk_collision_object The collision object representing the tree trunk, to check for collisions between the robot and the tree.
 * @param parameters A struct containing parameters for the TSP-over-PRM algorithm, such as the number of samples to take and the number of nearest neighbors to connect.
 * @param observer An observer object with callbacks for observing the progress of the algorithm.
 *
 * @return RobotPath The planned path, including all the states the robot should go through to visit all fruits.
 */
RobotPath plan_path_tsp_over_prm(
	const RobotState &initial_state,
	const std::vector<math::Vec3d> &fruit_positions,
	const robot_model::RobotModel &robot_model,
	const fcl::CollisionObjectd &tree_trunk_collision_object,
	const TspOverPrmParameters &parameters,
	const std::optional<TspOverPrmObserver> &observer
) {
}

REGISTER_VISUALIZATION(tsp_over_prm) {
	using namespace mgodpl;
	using namespace visualization;

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

	// Allocate an empty prm.
	TwoTierMultigoalPRM prm;

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
	add_and_connect_roadmap_node(start_state, prm, n_neighbours, std::nullopt, motion_collides);

	// The actors for the last-vizualized robot configuration sample(s), so we can remove them later.
	std::vector<vizualisation::RobotActors> sample_viz;

	// Index to track which goal to sample next in the visualization.
	size_t goal_being_sampled = 0;

	// Store the goal sample vertex nodes, with a separate sub-vector for each goal.
	// Start with a vector of empty vectors.
	std::vector<PRMGraph::vertex_descriptor> goal_nodes;

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
	std::vector<std::vector<PRMGraph::vertex_descriptor> > predecessor_lookup;
	std::vector<PRMGraph::vertex_descriptor> start_to_goals_predecessors;

	// Look up the link IDs for the base and end effector.
	robot_model::RobotModel::LinkId base_link = robot.findLinkByName("flying_base");
	robot_model::RobotModel::LinkId end_effector_link = robot.findLinkByName("end_effector");

	VtkLineSegmentsVisualization distance_edges_viz(0.5, 0.5, 0.5);
	viewer.addActor(distance_edges_viz.getActor());

	std::vector<size_t> group_sizes(fruit_positions.size(), 0);

	std::optional<RobotPath> unoptimized_path;

	/// The goal sample index that's being distanced and predecessor-looked-up.
	size_t next_goal_sample_index = 0;

	std::function viz_sampled_state = [&](const RobotState &state, bool added) {
		// Pick a color based on whether it collides.
		math::Vec3d color = added ? math::Vec3d{0.0, 1.0, 0.0} : math::Vec3d{1.0, 0.0, 0.0};

		// Compute the forward kinematics.
		auto fk = forwardKinematics(robot, state.joint_values, 0, state.base_tf);

		// Visualize the robot state.
		sample_viz.push_back(vizualisation::vizualize_robot_state(viewer, robot, fk, color));
	};

	std::function viz_edge = [&](const PRMGraph::vertex_descriptor &source,
	                             const PRMGraph::vertex_descriptor &target,
	                             bool added) {
		if (added) {
			edges.emplace_back(
				prm.graph[source].state.base_tf.translation,
				prm.graph[target].state.base_tf.translation
			);
		}
	};

	std::function viz_goal_edge = [&](const PRMGraph::vertex_descriptor &source,
	                                  const PRMGraph::vertex_descriptor &target,
	                                  bool added) {
		if (added) {
			goal_edges.emplace_back(
				prm.graph[source].state.base_tf.translation,
				prm.graph[target].state.base_tf.translation
			);
		}
	};

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

			// Execute different logic depending on whether the infrastructure nodes are full or not.
			if (boost::num_vertices(prm.graph) < max_samples) {
				InfrastructureSampleHooks hooks{
					.on_sample = viz_sampled_state,
					.add_roadmap_node_hooks = AddRoadmapNodeHooks{
						.on_edge_considered = viz_edge
					}
				};

				sample_and_connect_infrastucture_node(
					prm,
					n_neighbours,
					sample_uniform,
					state_collides,
					motion_collides,
					hooks
				);

				prm_edges.updateLine(edges);
			} else if (goal_being_sampled < fruit_positions.size()) {
				std::function goal_sample = [&]() {
					return genGoalStateUniform(
						rng,
						fruit_positions[goal_being_sampled],
						robot,
						base_link,
						end_effector_link
					);
				};

				InfrastructureSampleHooks hooks{
					.on_sample = viz_sampled_state,
					.add_roadmap_node_hooks = AddRoadmapNodeHooks{
						.on_edge_considered = viz_goal_edge
					}
				};

				auto new_goal_nodes = sample_and_connect_goal_states(
					prm,
					{.k_neighbors = n_neighbours, .max_valid_samples = max_samples_per_goal, .max_attempts = 100},
					goal_being_sampled,
					goal_sample,
					state_collides,
					motion_collides,
					hooks
				);

				prm_goal_edges.updateLine(goal_edges);

				goal_nodes.insert(goal_nodes.end(), new_goal_nodes.begin(), new_goal_nodes.end());
				group_sizes[goal_being_sampled] = new_goal_nodes.size();
				if (goal_being_sampled == fruit_positions.size()) {
					group_index_table = GroupIndexTable(group_sizes);
				} else {
					++goal_being_sampled;
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
