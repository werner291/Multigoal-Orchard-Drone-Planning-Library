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
#include "nearest_neighbours/NearestNeighborsGNAT.h"

namespace mgodpl {
	// Define the graph type: an undirected graph with the defined vertex properties
	using PRMGraph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, RobotState,
		boost::property<boost::edge_weight_t, double> >;

	// A spatial index for nearest neighbors in the PRM graph.
	using PRMGraphSpatialIndex = ompl::NearestNeighborsGNAT<std::pair<RobotState, PRMGraph::vertex_descriptor> >;

	/**
	 * A set of hooks for adding a roadmap node.
	 */
	struct AddRoadmapNodeHooks {
		/// A function called when an edge is considered, with the source and target states, and a boolean decision (true if added, false if not).
		std::function<void(
			std::pair<const RobotState &, const PRMGraph::vertex_descriptor &>,
			std::pair<const RobotState &, const PRMGraph::vertex_descriptor &>,
			bool)> on_edge_considered;
	};

	/**
	 * A set of hooks for sampling and connecting infrastructure nodes.
	 */
	struct PrmBuildHooks {
		// A callback for when a sample is taken, with the state and a boolean indicating whether it was added.
		std::function<void(const RobotState &state, bool added)> on_sample;
		// Sub-hooks for connecting the roadmap node to the nearest neighbors.
		std::optional<AddRoadmapNodeHooks> add_roadmap_node_hooks;
	};

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
	 * @brief Filters the distances towards goal nodes from the distances vector.
	 *
	 * @param goal_nodes	A vector of vertex descriptors representing the goal nodes in the PRM graph.
	 * @param distances		A vector containing the distances between nodes in the PRM graph, indexed by the vertex descriptors.
	 *
	 * @return A vector of distances that correspond to the goal nodes.
	 */
	std::vector<double> filter_goal_distances_vector(
		const std::vector<PRMGraph::vertex_descriptor> &goal_nodes,
		const std::vector<double> &distances
	);

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
		std::optional<PrmBuildHooks> infrastructure_sample_hooks;
		/// Hooks for sampling and connecting goal nodes.
		std::optional<GoalSampleHooks> goal_sample_hooks;
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
