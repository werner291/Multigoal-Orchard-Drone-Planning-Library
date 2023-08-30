// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 30-8-23.
//

#include <queue>
#include <range/v3/view/zip.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include "any_to_any_dijkstra.h"

namespace mgodpl::any_to_any_dijkstra {

	/**
	 * @brief Compare two candidate edges, used to sort the priority queue.
	 *
	 * @param a 	The first candidate edge.
	 * @param b 	The second candidate edge.
	 * @return 		Whether the first candidate edge has a higher cost than the second candidate edge.
	 */
	bool candidate_priority_queue_compare(const mgodpl::any_to_any_dijkstra::CandidateEdge &a,
										  const mgodpl::any_to_any_dijkstra::CandidateEdge &b) {
		return a.cost > b.cost;
	}

	using PQueue = std::priority_queue<CandidateEdge, std::vector<CandidateEdge>, std::function<bool(const CandidateEdge &, const CandidateEdge &)>>;

	/**
	 * @brief Initialize a priority queue of candidates using the start vertices.
	 *
	 * This function initializes a priority queue of candidates using the start vertices,
	 * where the cost is zero for each one and the incoming edge is nullopt.
	 *
	 * @param start_points  The start vertices.
	 * @return 			    The priority queue of candidates.
	 */
	PQueue init_queue(const std::vector<Vertex> &start_points) {

		// Initialize a priority queue of candidates.
		PQueue queue(candidate_priority_queue_compare);

		// Put the start vertices into the queue.
		for (const auto &start_point: start_points) {
			// Candidate edges incoming to the start vertex have no cost.
			// There is effectively an implicit "start" vertex, signaled by a nullopt "from" vertex.
			queue.push({
							   .to_vertex = start_point,
							   .from_vertex = {},
							   .cost = 0.0
			});
		}

		return queue;

	}

	/**
	 * @brief Retrace the path from a group of goal vertices to the start vertex.
	 *
	 * This function retraces the path from a group of goal vertices to the start vertex,
	 * starting from the goal vertex with the lowest cost, and then following the parent
	 * vertices until we reach a vertex with no parent.
	 *
	 * Ideally this would be the start vertex, but if the goal vertex group is not connected
	 * to the start vertex, then it will be an empty path.
	 *
	 * @param parents 			The parent vertices for each vertex in the graph.
	 * @param goal_group 		The goal vertex group.
	 * @return 					The path from the goal vertex group to the start vertex; empty vector if no path exists.
	 */
	std::vector<Vertex> retrace_path(const std::vector<Parent> &parents,
									 const std::vector<Vertex>& goal_group) {

		std::vector<Vertex> path;

		// Find the goal vertex with the lowest cost.
		auto best_goal_vertex = std::min_element(goal_group.begin(), goal_group.end(), [&](auto a, auto b) {
			return parents[a].cost < parents[b].cost;
		});

		std::optional<Vertex> current_vertex = best_goal_vertex == goal_group.end() ? std::nullopt : std::make_optional(*best_goal_vertex);

		while (current_vertex.has_value()) {

			// Add the current vertex to the path.
			path.push_back(current_vertex.value());

			current_vertex = parents[current_vertex.value()].best_parent;

		}

		return path;
	}

	/**
	 * @brief Retrace the paths from the goal vertex groups to the start vertices.
	 *
	 * This function retrace the paths from the goal vertex groups to the start vertices,
	 * starting from the goal vertex with the lowest cost in each group, and then following
	 * the parent vertices until we reach a vertex with no parent.
	 *
	 * Ideally this would be the start vertex, but if the goal vertex group is not connected
	 * to the start vertex, then it will be an empty path.
	 *
	 * In the returned vector, each entry corresponds to a goal vertex group. Each entry is
	 * a vector of vertices, starting with the start vertex and ending with the goal vertex.
	 *
	 * An empty vector means that no path exists.
	 *
	 * @param start_points  The start vertices.
	 * @param goal_groups   The goal vertex groups.
	 * @param parents       The parent vertices for each vertex in the graph.
	 *
	 * @return              The paths from the goal vertex groups to the start vertices; empty vector if no path exists.
	 */
	std::vector<std::vector<Vertex>> retrace_paths(const std::vector<Vertex> &start_points,
															  const std::vector<std::vector<Vertex>> &goal_groups,
															  const std::vector<Parent> &parents) {
		return goal_groups | ranges::views::transform([&](const auto &goal_group) {
			return retrace_path(parents, goal_group);
		}) | ranges::to<std::vector>();
	}

	/**
	 * @brief Visit a vertex in the graph and perform necessary operations.
	 *
	 * This function visits a vertex in the given graph, updates its parent information,
	 * and expands the vertex by adding its neighbors to a priority queue for further exploration.
	 *
	 * @param graph The graph containing the vertices and edges.
	 * @param distance A function that computes the distance between two vertices.
	 * @param parents A vector containing parent information for each vertex in the graph.
	 * @param queue The priority queue used for vertex exploration.
	 * @param candidate The candidate edge connecting the current vertex to its neighbor.
	 * @param parent Reference to the parent information of the current vertex.
	 */
	void visit_vertex(const Graph &graph,
					  const std::function<double(const Vertex &, const Vertex &)> &distance,
					  const std::vector<Parent> &parents,
					  PQueue &queue,
					  const CandidateEdge &candidate,
					  Parent &parent) {

		// Visit the node.
		parent.visited = true;
		parent.best_parent = candidate.from_vertex;
		parent.cost = candidate.cost;

		assert(parent.cost != 0.0 || !parent.best_parent.has_value());

		// Expand the vertex by looking up its neighbors (boost graph)
		// and adding them to the queue.
		for (const auto &e : boost::make_iterator_range(boost::out_edges(candidate.to_vertex, graph))) {

			// Get the target vertex.
			Vertex target_vertex = boost::target(e, graph);

			// Look up the parent vertex.
			const Parent &target_parent = parents[target_vertex];

			// If we've already visited this vertex, then we can skip it.
			if (target_parent.visited) {
				continue;
			}

			// Compute the cost of the edge.
			double edge_cost = distance(candidate.to_vertex, target_vertex);

			// Add the edge to the queue.
			queue.push({
				.to_vertex = target_vertex,
				.from_vertex = candidate.to_vertex,
				.cost = candidate.cost + edge_cost
			});

		}

	}

	/**
	 * @brief Find paths from a set of start points to multiple goal groups in a graph.
	 *
	 * This function finds paths from a set of start points to multiple goal groups in the given graph.
	 * It performs a search using the PRM algorithm and returns a vector of vectors containing paths from
	 * each start point to the goal groups.
	 *
	 * The result vector will have the same length as the goal groups vector. Each entry in the result vector
	 * is a vector of vertices, starting with the start vertex and ending with the goal vertex.
	 *
	 * An empty vector means that no path exists.
	 *
	 * @param graph 		The graph containing vertices and edges for the search.
	 * @param distance 		A function that computes the distance between two vertices.
	 * @param start_points 	The vector of start vertices.
	 * @param goal_groups 	The vector of vectors containing goal vertices organized in groups.
	 * @return A vector of vectors of vertices representing paths from start points to goal groups.
	 */
	std::vector<std::vector<Vertex>> any_to_any(const Graph &graph,
														   const std::function<double(const Vertex &,
																					  const Vertex &)> &distance,
														   const std::vector<Vertex>& start_points,
														   const std::vector<std::vector<Vertex>>& goal_groups) {

		// Initialize a vector of "parent" vertices for each graph vertex.
		std::vector<Parent> parents(graph.m_vertices.size());

		// Create an initial queue from the start vertices.
		auto queue = init_queue(start_points);

		// Iterate over the queue until it's empty.
		while (!queue.empty()) {

			// Get the next vertex to expand.
			CandidateEdge candidate = queue.top();
			// Remove it from the queue.
			queue.pop();

			// Look up the vertex we're entering.
			Parent &parent = parents[candidate.to_vertex];

			// If we've already visited this vertex, then we can skip it.
			if (parent.visited) {
				continue;
			}

			visit_vertex(graph, distance, parents, queue, candidate, parent);

		}

		return retrace_paths(start_points, goal_groups, parents);
	}

}