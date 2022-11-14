#include <moveit/robot_state/robot_state.h>
#include "traveling_salesman.h"
#include "general_utilities.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_parameters.h"

#include <utility>
#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/view/enumerate.hpp>
#include <boost/range/irange.hpp>


typedef std::vector<std::vector<int64_t>> Int64DistanceMatrix;


std::tuple<Int64DistanceMatrix, size_t, size_t> mkOpenEndedDistanceMatrix(const std::function<double(size_t)> &from_start,
																		  const std::function<double(size_t, size_t)> &between,
																		  size_t n,
																		  const ompl::base::PlannerTerminationCondition &ptc) {

    Int64DistanceMatrix distance_matrix(n+2, std::vector<int64_t>(n+2));

    size_t start_state_index= n;
    size_t end_state_index= n + 1;// First, we build an (N+2)^2 square symmetric distance matrix.

    // The top-left NxN part of the matrix is filled with apple-to-apple distances,
    // where distance_matrix[i][j] contains the distance between apples[i] and apples[j].
    // Distance matrix is assumed symmetric
    for (size_t i : boost::irange<size_t>(0,n)) {
        for (size_t j : boost::irange<size_t>(i,n)) {
            distance_matrix[i][j] = (int64_t) (between(i,j) * 1000.0);
            // Lower triangle is filled by symmetry
            distance_matrix[j][i] = distance_matrix[i][j];
        }
		checkPtc(ptc);
    }

    // The distance_matrix[N] column contains the distance between the start state and the apple.
    // Also, the distance_matrix[..][N] row is built by symmetry.

    for (size_t i : boost::irange<size_t>(0,n)) {
        distance_matrix[i][n] = (int64_t) (from_start(i) * 1000.0);
        distance_matrix[n][i] = distance_matrix[i][n];
    }

    // Traveling from the start state to itself is free.
    distance_matrix[start_state_index][start_state_index] = 0;

    // We add a "dummy" node to visit as the required end position, with a 0-cost edge to every other node.
    for (size_t i : boost::irange<size_t>(0,n+2)) {
        distance_matrix[i][end_state_index] = 0;
        distance_matrix[end_state_index][i] = 0;
    }
    
    return std::make_tuple(
            distance_matrix,
            start_state_index,
            end_state_index
            );
}

std::vector<size_t> tsp_open_end(
		const std::function<double(size_t)> &from_start,
		const std::function<double(size_t, size_t)> &between,
		size_t n,
		const ompl::base::PlannerTerminationCondition &ptc) {
    
    const auto &[distance_matrix, start_state_index, end_state_index] =
            mkOpenEndedDistanceMatrix(from_start, between, n, ptc);

    // This allows us to translate OR-tools internal indices into the indices of the table above.
    operations_research::RoutingIndexManager manager((int) n+2, 1,
                                                     { operations_research::RoutingIndexManager::NodeIndex {(int) start_state_index } },
                                                     { operations_research::RoutingIndexManager::NodeIndex {(int) end_state_index } });

    // Build a routing model, and register the distance matrix.
    operations_research::RoutingModel routing(manager);
    routing.SetArcCostEvaluatorOfAllVehicles(routing.RegisterTransitMatrix(distance_matrix));

    operations_research::RoutingSearchParameters searchParameters = operations_research::DefaultRoutingSearchParameters();
    searchParameters.set_first_solution_strategy(operations_research::FirstSolutionStrategy::PATH_CHEAPEST_ARC);

    const operations_research::Assignment* solution = routing.SolveWithParameters(searchParameters);

    // Translate the internal ordering into an ordering on the apples.
    std::vector<size_t> ordering;

    for (int64_t index = routing.Start(0); !routing.IsEnd(index); index = solution->Value(routing.NextVar(index))) {

        size_t node_index = (size_t) manager.IndexToNode(index).value();

        // Exclude the dummy start-and-end nodes.
        if (node_index != start_state_index && node_index != end_state_index) {
            ordering.push_back(node_index);
        }
    }

    return ordering;
}


std::vector<std::pair<size_t, size_t>> flatten_indices(const std::vector<size_t> &sizes) {
    std::vector<std::pair<size_t, size_t>> index_pairs;
    for (size_t i : boost::irange<size_t>(0,sizes.size())) {
        for (size_t j : boost::irange<size_t>(0,sizes[i])) {
            index_pairs.emplace_back(i,j);
        }
    }
    return index_pairs;
}

std::vector<std::pair<size_t, size_t>>
tsp_open_end_grouped(const std::function<double(std::pair<size_t, size_t>)> &from_start,
					 const std::function<double(std::pair<size_t, size_t>, std::pair<size_t, size_t>)> &between,
					 const std::vector<size_t> &sizes,
					 ompl::base::PlannerTerminationCondition &ptc) {

    // Sum up all sizes
    auto index_lookup = flatten_indices(sizes);

    size_t n = index_lookup.size();

    std::cout << "Building TSP distance matrix." << std::endl;

    const auto &[distance_matrix, start_state_index, end_state_index] =
            mkOpenEndedDistanceMatrix(
                    [&](size_t i) {
                        return from_start(index_lookup[i]);
                    },
                    [&](size_t i, size_t j) {
                        return between(index_lookup[i], index_lookup[j]);
                    },
                    n,
					ptc);

    std::cout << "Solving the routing problem..." << std::endl;

    // This allows us to translate OR-tools internal indices into the indices of the table above.
    operations_research::RoutingIndexManager manager((int) n+2, 1,
                                                     { operations_research::RoutingIndexManager::NodeIndex {(int) start_state_index } },
                                                     { operations_research::RoutingIndexManager::NodeIndex {(int) end_state_index } });

    // Build a routing model, and register the distance matrix.
    operations_research::RoutingModel routing(manager);
    routing.SetArcCostEvaluatorOfAllVehicles(routing.RegisterTransitMatrix(distance_matrix));

    // Set up the disjunctions, to allow the router to only visit every group once.
    size_t last_index = 0;
    std::vector<std::pair<size_t, size_t>> index_pairs;
    for (size_t i : boost::irange<size_t>(0,sizes.size())) {
        std::vector<int64_t> disjunction_indices;
        for (size_t j : boost::irange<size_t>(0,sizes[i])) {
            disjunction_indices.emplace_back(last_index++);
        }
        if (disjunction_indices.size() >= 2) {
            routing.AddDisjunction(disjunction_indices);
        }
    }

    operations_research::RoutingSearchParameters searchParameters = operations_research::DefaultRoutingSearchParameters();
    searchParameters.set_first_solution_strategy(operations_research::FirstSolutionStrategy::PATH_CHEAPEST_ARC);

    const operations_research::Assignment* solution = routing.SolveWithParameters(searchParameters);

    // Translate the internal ordering into an ordering on the apples.
    std::vector<std::pair<size_t, size_t>> ordering;

    for (int64_t index = routing.Start(0); !routing.IsEnd(index); index = solution->Value(routing.NextVar(index))) {

        size_t node_index = (size_t) manager.IndexToNode(index).value();

        // Exclude the dummy start-and-end nodes.
        if (node_index != start_state_index && node_index != end_state_index) {
            ordering.push_back(index_lookup[node_index]);
        }
    }

    return ordering;
}