// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 30-8-23.
//

#ifndef MGODPL_ANY_TO_ANY_DIJKSTRA_H
#define MGODPL_ANY_TO_ANY_DIJKSTRA_H

#include <vector>
#include <optional>
#include <cmath>
#include "planners/MultigoalPrmStar.h"

namespace mgodpl::any_to_any_dijkstra {

	// TODO: Replace this with the proper Boost type.
	using Vertex = PRMCustom::Vertex;
	using Graph = PRMCustom::Graph;

	/**
	 * A vertex in the roadmap with a parent vertex and a cost.
	 */
	struct Parent {
		// The best parent vertex. (nullopt if this is the start vertex or if the node has not been visited.)
		std::optional<PRMCustom::Vertex> best_parent = std::nullopt;
		// The cost of the best path to this vertex. (INFINITY if this node has not been visited.)
		double cost = INFINITY;
		// Whether this vertex has been visited.
		bool visited = false;
	};


	/**
	 * A candidate edge to explore.
	 */
	struct CandidateEdge {
		// The vertex we're coming from. (nullopt if this is the start vertex.)
		PRMCustom::Vertex to_vertex;
		// The vertex we're going to.
		std::optional<PRMCustom::Vertex> from_vertex;
		// The cost of the edge.
		double cost = INFINITY;
	};

	std::vector<std::vector<PRMCustom::Vertex>> any_to_any(
			const PRMCustom::Graph& graph,
			const std::function<double(const PRMCustom::Vertex&, const PRMCustom::Vertex&)>& distance,
			const std::vector<PRMCustom::Vertex>& start_points,
			const std::vector<std::vector<PRMCustom::Vertex>>& goal_groups);


}

#endif //MGODPL_ANY_TO_ANY_DIJKSTRA_H
