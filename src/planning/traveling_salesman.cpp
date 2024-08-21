// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "traveling_salesman.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_parameters.h"

#include <utility>
#include <boost/range/irange.hpp>

// OR-tools assumes integer distances. This isn't too big of a deal, because if we use i64, then we can just multiply
// the distances by some large-ish number and round without loosing too much precision.
using Int64DistanceMatrix = std::vector<std::vector<int64_t> >;

// Since OR-tools assumes integers, we multiply the double distances by this factor before rounding.
const double DOUBLE_TO_INTEGER_MULTIPLER = 1000.0;

/**
 * Fill in the top-left NxN part of the distance matrix with the distances between the goals.
 *
 * Note that the distance function is assumed to be symmetric, i.e., distance_matrix[i][j] = distance_matrix[j][i].
 *
 * @param between			A function that, given two goal indices, returns the distance between those goals.
 * @param n					The number of goals.
 * @param distance_matrix	The distance matrix to fill in.
 */
void fill_goal_to_goal_matrix(const std::function<double(size_t, size_t)> &between,
                              size_t n,
                              Int64DistanceMatrix distance_matrix) {
	// The top-left NxN part of the matrix is filled with goal-to-goal distances,
	// where distance_matrix[i][j] contains the distance between apples[i] and apples[j].
	// Distance is assumed symmetric
	for (size_t i: boost::irange<size_t>(0, n)) {
		for (size_t j: boost::irange<size_t>(i, n)) {
			distance_matrix[i][j] = (int64_t) (between(i, j) * DOUBLE_TO_INTEGER_MULTIPLER);
			// Lower triangle is filled by symmetry
			distance_matrix[j][i] = distance_matrix[i][j];
		}
	}
}

/**
 * Fill in the N-th row and column of the distance matrix with the distances between the start state and the goals.
 *
 * @param from_start	A function that, given a goal index, returns the distance between the start state and that goal.
 * @param n		The number of goals.
 * @param distance_matrix	The distance matrix to fill in.
 */
void fill_start_distance_matrix(const std::function<double(size_t)> &from_start,
                                size_t n,
                                Int64DistanceMatrix distance_matrix) {
	for (size_t i: boost::irange<size_t>(0, n)) {
		distance_matrix[i][n] = (int64_t) (from_start(i) * DOUBLE_TO_INTEGER_MULTIPLER);
		distance_matrix[n][i] = distance_matrix[i][n];
	}
}

/**
 * Fill in the N+1-th row and column of the distance matrix with 0, or the distance between the goal i and an imaginary end state.
 *
 * @param n					The number of goals.
 * @param distance_matrix	The distance matrix to fill in.
 * @param end_state_index	The index of the end state.
 */
void fill_dummy_end_distances(size_t n, Int64DistanceMatrix distance_matrix, size_t end_state_index) {
	for (size_t i: boost::irange<size_t>(0, n + 2)) {
		distance_matrix[i][end_state_index] = 0;
		distance_matrix[end_state_index][i] = 0;
	}
}

/**
 * Create an open-ended integer distance matrix for the TSP problem,
 * based on inter-goal and start-to-goal distances.
 *
 * Assuming N goals, the distance matrix will be an (N+2)x(N+2) square matrix, symmetric along the diagonal.
 *
 * If D is the distance matrix, then:
 *	- For i,j < N, D[i][j] is the distance between goal i and goal j.
 *	- For i < N, D[i][N] is the distance between the start state and goal i.
 *	- For i < N, D[i][N+1] is 0, or the distance between the goal i and an imaginary end state.
 *
 *	Since the matrix is symmetric, D[j][i] = D[i][j] for all i,j < N.
 *	Also, we assume that D[i][i] = 0 for all i < N.
 *
 * @param from_start	A function that, given a goal index, returns the distance between the start state and that goal.
 * @param between		A function that, given two goal indices, returns the distance between those goals.
 * @param n				The number of goals.
 * @return			    A tuple containing the (n+2)^2 distance matrix, the index of the start state, and the index of the end state.
 */
std::tuple<Int64DistanceMatrix, size_t, size_t> mkOpenEndedDistanceMatrix(
	const std::function<double(size_t)> &from_start,
	const std::function<double(size_t, size_t)> &between,
	size_t n) {
	// Allocate a square matrix of size (n+2)x(n+2) to store the distances.
	Int64DistanceMatrix distance_matrix(n + 2, std::vector<int64_t>(n + 2));

	// We map the start state to the index N,
	size_t start_state_index = n;
	// and the end state to the index N+1.
	size_t end_state_index = n + 1;

	// We fill in the goal-to-goal distances.
	fill_goal_to_goal_matrix(between, n, distance_matrix);

	// The distance_matrix[N] column contains the distance between the start state and the apple.
	// Also, the distance_matrix[..][N] row is built by symmetry.
	fill_start_distance_matrix(from_start, n, distance_matrix);

	// Traveling from the start state to itself is free.
	distance_matrix[start_state_index][start_state_index] = 0;

	// We add a "dummy" node to visit as the required end position, with a 0-cost edge to every other node.
	fill_dummy_end_distances(n, distance_matrix, end_state_index);

	// Return the matrix itself, and the indices of the start and end states.
	return std::make_tuple(
		distance_matrix,
		start_state_index,
		end_state_index
	);
}

/**
 * Determine an approximately optimal ordering of a given set of items/indices, and distances between them.
 *
 * This function assumes that all items are to be visited, and that the tour starts from some implicit item,
 * visits all items in the set, and terminates at an arbitrary item in the set.
 *
 * Note that this function only works through indices: it assumes that the parent context knows how to map these indices
 * to something more meaningful, like robot states.
 *
 * @param from_start	A function that gives a distance between a given goal index and some assumed start item.
 * @param between		A function that gives a distance between two given goal indices.
 * @param n				The number of goals in the set (all indices will be in the range 0..n-1).
 * @return				The tour, represented as a vector of indices.
 */
std::vector<size_t> tsp_open_end(
	const std::function<double(size_t)> &from_start,
	const std::function<double(size_t, size_t)> &between,
	size_t n) {
	// If there are no goals, there is no tour; just return an empty vector.
	if (n == 0) {
		return {};
	}

	// If there is only one goal, there is no choice to make; just return the single goal.
	if (n == 1) {
		return {0};
	}

	// Otherwise, build a distance matric that OR-tools can use.
	const auto &[
		distance_matrix,
		start_state_index,
		end_state_index] = mkOpenEndedDistanceMatrix(from_start, between, n);

	// Below is mostly OR-tools boilerplate code to solve the TSP problem.

	// Build a routing index manager that keeps track of what does what.
	operations_research::RoutingIndexManager manager((int) n + 2,
	                                                 // n+2 nodes: n goals, 1 (implicit) start state, 1 (dummy) end state.
	                                                 1,
	                                                 // 1 vehicle; i.e. the robot itself.
	                                                 {
		                                                 // The start state is the index n.
		                                                 operations_research::RoutingIndexManager::NodeIndex{
			                                                 (int) start_state_index
		                                                 }
	                                                 },
	                                                 {
		                                                 // The end state is the imaginary node that has a 0-cost edge to every other node.
		                                                 operations_research::RoutingIndexManager::NodeIndex{
			                                                 (int) end_state_index
		                                                 }
	                                                 });

	// Build a routing model, and register the distance matrix we just built.
	operations_research::RoutingModel routing(manager);
	routing.SetArcCostEvaluatorOfAllVehicles(routing.RegisterTransitMatrix(distance_matrix));

	// Set up the search parameters. (TODO: investigate these parameters more.)
	operations_research::RoutingSearchParameters searchParameters =
			operations_research::DefaultRoutingSearchParameters();
	searchParameters.set_first_solution_strategy(operations_research::FirstSolutionStrategy::PATH_CHEAPEST_ARC);

	// Run the solver.
	const operations_research::Assignment *solution = routing.SolveWithParameters(searchParameters);

	// Sanity check: the solution should not be null; i.e. this problem should always be solvable. If not, there's something wrong.
	assert(solution != nullptr);

	// Translate the internal ordering into an ordering on the apples.
	std::vector<size_t> ordering;

	for (int64_t index = routing.Start(0); !routing.IsEnd(index); index = solution->Value(routing.NextVar(index))) {
		size_t node_index = (size_t) manager.IndexToNode(index).value();

		// Exclude the dummy start-and-end nodes.
		if (node_index != start_state_index && node_index != end_state_index) {
			ordering.push_back(node_index);
		}
	}

	// Return our ordering.
	return ordering;
}

/**
 * Given: a vector of sizes, where each element is the number of items in a group, produce a vector of pairs of indices,
 * where each pair refers to one item from one of the groups.
 *
 * For instance, a vector of sizes {2, 3} would produce the vector {(0, 0), (0, 1), (1, 0), (1, 1), (1, 2)}.
 *
 * This is useful to create, for instance, a "flat" vector of indices for the discrete neighborhood TSP problem.
 *
 * @param sizes		A vector of the number of items in each group.
 * @return		A vector of pairs of indices, where each pair refers to one item from one of the groups.
 */
std::vector<std::pair<size_t, size_t> > flatten_indices(const std::vector<size_t> &sizes) {
	std::vector<std::pair<size_t, size_t> > index_pairs;
	for (size_t i: boost::irange<size_t>(0, sizes.size())) {
		for (size_t j: boost::irange<size_t>(0, sizes[i])) {
			index_pairs.emplace_back(i, j);
		}
	}
	return index_pairs;
}

/**
 * Build the disjunction constraints for OR-tools.
 *
 * That is: we will provide OR tools with multiple sets of indices, where each set contains the indices of items in a group.
 * This communicates to OR tools that we only need to visit one item from each group.
 *
 * Note: this function should match up with the way the indices are flattened in the `flatten_indices` function.
 *
 * @param sizes		A vector of the number of items in each group.
 * @param routing	The routing model to add the disjunctions to.
 */
void build_disjunctions(const std::vector<size_t> &sizes, operations_research::RoutingModel &routing) {
	// Keep track of the number of items we've processed; this corresponds to the number
	// of iterations in the inner loop, as well as the items in flattened_indices.

	size_t last_index = 0;

	// For each group, we add a disjunction constraint that requires the router to visit at least one item from that group.
	for (size_t i: boost::irange<size_t>(0, sizes.size())) {
		// Build up a vector of indices for the items in this group.
		std::vector<int64_t> disjunction_indices;
		for (size_t j: boost::irange<size_t>(0, sizes[i])) {
			disjunction_indices.emplace_back(last_index++); // Add the index of the item to the disjunction.
		}

		// If there are at least two items in the group, we add a disjunction. Otherwise, the disjunction is trivially satisfied.
		if (disjunction_indices.size() >= 2) {
			routing.AddDisjunction(disjunction_indices); // Add the disjunction to the routing model.
		}
	}
}

std::vector<std::pair<size_t, size_t> >
tsp_open_end_grouped(const std::function<double(std::pair<size_t, size_t>)> &from_start,
                     const std::function<double(std::pair<size_t, size_t>, std::pair<size_t, size_t>)> &between,
                     const std::vector<size_t> &sizes) {
	/*
	 * In the discrete neighborhood TSP problem, we have a set of N neighborhoods, each with a certain number of items,
	 * as well as a distance function between items (both within and across neighborhoods).
	 *
	 * We want to find an approximately optimal ordering of all items, such that we visit at least one item from each
	 * neighborhood, and such that the total distance traveled is minimized.
	 */

	// We first flatten the indices of the items, so that a single intiger index can be used to refer to any item regardless of group.
	auto index_lookup = flatten_indices(sizes);

	// We then determine the number of items in the flattened index set.
	size_t n = index_lookup.size();

	// Build a distance matrix that OR-tools can use. Note that this is just the same as the regular, non-grouped TSP problem.
	const auto &[distance_matrix, start_state_index, end_state_index] =
			mkOpenEndedDistanceMatrix(
				[&](size_t i) {
					return from_start(index_lookup[i]);
				},
				[&](size_t i, size_t j) {
					return between(index_lookup[i], index_lookup[j]);
				},
				n);

	// This allows us to translate OR-tools internal indices into the indices of the table above.
	operations_research::RoutingIndexManager manager((int) n + 2,
	                                                 // n+2 nodes: n goals, 1 (implicit) start state, 1 (dummy) end state.
	                                                 1,
	                                                 {
		                                                 // The start state is the index n.
		                                                 operations_research::RoutingIndexManager::NodeIndex{
			                                                 (int) start_state_index
		                                                 }
	                                                 },
	                                                 {
		                                                 // The end state is the imaginary node that has a 0-cost edge to every other node.
		                                                 operations_research::RoutingIndexManager::NodeIndex{
			                                                 (int) end_state_index
		                                                 }
	                                                 });

	// Build a routing model, and register the distance matrix.
	operations_research::RoutingModel routing(manager);
	routing.SetArcCostEvaluatorOfAllVehicles(routing.RegisterTransitMatrix(distance_matrix));

	// Set up the disjunctions, to allow the router to only visit every group once.
	build_disjunctions(sizes, routing);

	// Set up the search parameters. (TODO: investigate these parameters more.)
	operations_research::RoutingSearchParameters searchParameters =
			operations_research::DefaultRoutingSearchParameters();
	searchParameters.set_first_solution_strategy(operations_research::FirstSolutionStrategy::PATH_CHEAPEST_ARC);

	// Run the solver.
	const operations_research::Assignment *solution = routing.SolveWithParameters(searchParameters);

	// Translate the internal ordering into an ordering on the apples.
	std::vector<std::pair<size_t, size_t> > ordering;

	for (int64_t index = routing.Start(0); !routing.IsEnd(index); index = solution->Value(routing.NextVar(index))) {
		// Translate OR-Tools' internal indices to indices in the flattened index set.
		size_t node_index = (size_t) manager.IndexToNode(index).value();

		// Exclude the dummy start-and-end nodes.
		if (node_index != start_state_index && node_index != end_state_index) {
			// Add to the ordering, after mapping back to the original indices.
			ordering.push_back(index_lookup[node_index]);
		}
	}

	// Return our ordering.
	return ordering;
}
