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
} // mgodpl

#endif //TSP_OVER_PRM_H
