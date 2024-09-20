module;

// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#include <cstddef>
#include <vector>
#include <optional>

#include "RobotState.h"
#include "RobotPath.h"

export module rrt;

namespace mgodpl {

	/**
	 * \struct RRTNode
	 * \brief Represents a node in the Rapidly-exploring Random Tree (RRT).
	 */
	export struct RRTNode {
		RobotState state; ///< The state of the robot at this node.
		size_t parent_index; ///< The index of the parent node in the tree.
	};

	/**
	 * \brief Finds the nearest node in the tree to the new state.
	 *
	 * \param nodes A vector of RRTNode representing the nodes in the tree.
	 * \param new_state The new state to find the nearest node to.
	 * \param distance A function that computes the distance between two states.
	 * \return The index of the nearest node in the tree.
	 */
	size_t linear_nearest(const std::vector<RRTNode> &nodes, const RobotState &new_state, const DistanceFn &distance) {
		assert(!nodes.empty());

		size_t nearest_index = 0; // Find the nearest node in the tree to the new state
		double nearest_distance = distance(new_state, nodes[0].state);

		for (size_t i = 1; i < nodes.size(); ++i) {
			double d = distance(new_state, nodes[i].state);
			if (d < nearest_distance) {
				nearest_distance = d;
				nearest_index = i;
			}
		}
		return nearest_index;
	}

	/**
	 * A callback function that is called after each iteration of the RRT algorithm when the tree is expanded.
	 *
	 * The `nodes` parameter contains the nodes in the tree after the expansion:
	 * - The last node is the new node added.
	 * - The root of the tree is nodes.front().
	 *
	 * \param nodes The nodes in the tree after the expansion; the last node is the new node added.
	 * \return True if the RRT algorithm should stop, false otherwise.
	 */
	export using RRTCallbackFn = std::function<bool(const std::vector<RRTNode> &nodes)>;

	/**
	 * \brief Performs the Rapidly-exploring Random Tree (RRT) algorithm to find a path from the root state.
	 *
	 * The RRT algorithm builds a tree of states starting from the root state. At each iteration, a new random state is sampled;
	 * if it can be connected to the nearest state in the tree without collision, it is added to the tree.
	 *
	 * Hooking instructions:
	 * - The `tree_expanded` callback function is called after each iteration when the tree is expanded.
	 * - The `collides` will be called for every sample, so can be used to introspect the sampling process.
	 * - The `motion_collides` will be called for every sample, so can be used to introspect which motions are considered.
	 *
	 * \param root The starting state of the robot.
	 * \param sample_state A function that samples a random state.
	 * \param collides A function that checks if a state is in collision.
	 * \param motion_collides A function that checks if the motion between two states is in collision.
	 * \param distance A function that computes the distance between two states.
	 * \param max_iterations The maximum number of iterations to run the algorithm.
	 * \param tree_expanded A callback function that is called after each iteration when the tree is expanded.
	 */
	export void rrt(const RobotState &root,
					const std::function<RobotState()> &sample_state,
					const std::function<bool(const RobotState &)> &collides,
					const std::function<bool(const RobotState &,
											 const RobotState &)> &motion_collides,
					const DistanceFn &distance,
					size_t max_iterations,
					const RRTCallbackFn &tree_expanded) {

		// Initialize the tree with the start node
		std::vector<RRTNode> nodes{
				{root, 0}
		};

		// Main loop of the RRT algorithm
		for (int iteration = 0; iteration < max_iterations; ++iteration) {
			// Sample a new random state
			auto new_state = sample_state();

			if (collides(new_state)) {
				continue;
			}

			size_t nearest_index = linear_nearest(nodes, new_state, distance);

			auto nearest_state = nodes[nearest_index].state;

			// Check if the motion from the nearest state to the new state is in collision
			if (motion_collides(nearest_state, new_state)) {
				continue;
			}

			// Add to the tree and parent to the nearest node
			nodes.push_back({new_state, nearest_index});

			// Callback:
			if (tree_expanded(nodes)) {
				break;
			}
		}
	}

	/**
	 * \brief Retraces the path from the last node to the root node.
	 *
	 * This function constructs the path by following the parent indices from the last node
	 * to the root node, and returns the path as a RobotPath object.
	 *
	 * Note: this path is in reverse order, starting from the last node and ending at the root node.
	 *
	 * \param nodes A vector of RRTNode representing the nodes in the tree.
	 * \return A RobotPath object containing the states from the last node to the root node.
	 */
	export RobotPath retrace(const std::vector<RRTNode> &nodes) {
		std::vector<RobotState> path_states;
		size_t current_index = nodes.size() - 1;

		do {
			path_states.push_back(nodes[current_index].state);
			current_index = nodes[current_index].parent_index;
		} while (current_index != 0);

		path_states.push_back(nodes[0].state);

		return RobotPath{.states = path_states};
	}

	/**
	 * Run RRT until finding an acceptable state (according to a std::function argument)
	 * and then return a path from that state to the root.
	 *
	 * \param root 				The starting state of the robot.
	 * \param sample_state 		A function that samples a random state.
	 * \param collides 			A function that checks if a state is in collision.
	 * \param motion_collides 	A function that checks if the motion between two states is in collision.
	 * \param distance 			A function that computes the distance between two states.
	 * \param max_iterations 	The maximum number of iterations to run the algorithm.
	 * \param state_acceptable 	A function that checks if a state is acceptable as a stating point for the path.
	 *
	 * \returns A path from the accepted state to the root, or nullopt if such a state wasn't found.
	 */
	export std::optional<RobotPath> rrt_path_to_acceptable(const RobotState &root,
														 const std::function<RobotState()> &sample_state,
														 const std::function<bool(const RobotState &)> &collides,
														 const std::function<bool(const RobotState &,
																				 const RobotState &)> &motion_collides,
														 const DistanceFn &distance,
														 size_t max_iterations,
														 const std::function<bool(const RobotState &)> &state_acceptable) {
		std::optional<RobotPath> path;
		rrt(
				root,
				sample_state,
				collides,
				motion_collides,
				distance,
				max_iterations,
				[&](const std::vector<RRTNode> &nodes) {
					if (state_acceptable(nodes.back().state)) {
						path = retrace(nodes);
						return true;
					} else {
						return false;
					}
				}
		);
		return path;
	}
}