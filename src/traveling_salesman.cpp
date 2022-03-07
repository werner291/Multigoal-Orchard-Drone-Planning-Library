#include <moveit/robot_state/robot_state.h>
#include "traveling_salesman.h"

#include <boost/range/algorithm/min_element.hpp>

std::vector<size_t>
apple_ordering_from_metric_ortools(const std::vector<Apple> &apples,
                                   const std::function<double(const Apple&)>& state_to_apple_distance_fn,
                                   const std::function<double(const Apple&, const Apple&)>& apple_to_apple_distance_fn) {

    // First, we build an (N+2)^2 square symmetric distance matrix.
    std::vector<std::vector<int64_t>> distance_matrix(apples.size() + 2, std::vector<int64_t>(apples.size() + 2));

    // The top-left NxN part of the matrix is filled with apple-to-apple distances,
    // where distance_matrix[i][j] contains the distance between apples[i] and apples[j].
    for (size_t i : boost::irange<size_t>(0,apples.size())) {
        for (size_t j : boost::irange<size_t>(0,apples.size())) {
            distance_matrix[i][j] = (int64_t) (apple_to_apple_distance_fn(apples[i],apples[j]) * 1000.0);
        }
    }

    // The distance_matrix[N] column contains the distance between the start state and the apple.
    // Also, the distance_matrix[..][N] row is built by symmetry.
    for (size_t i : boost::irange<size_t>(0,apples.size())) {
        distance_matrix[i][apples.size()] = (int64_t) (state_to_apple_distance_fn(apples[i]) * 1000.0);;
        distance_matrix[apples.size()][i] = distance_matrix[i][apples.size()];
    }

    size_t start_state_index = apples.size();

    // Traveling from the start state to itself is free.
    distance_matrix[start_state_index][start_state_index] = 0;

    size_t end_state_index = apples.size()+1;

    // We add a "dummy" node to visit as the required end position, with a 0-cost edge to every other node.
    for (size_t i : boost::irange<size_t>(0,apples.size()+2)) {
        distance_matrix[i][end_state_index] = 0;
        distance_matrix[end_state_index][i] = 0;
    }

    // This allows us to translate OR-tools internal indices into the indices of the table above.
    operations_research::RoutingIndexManager manager((int) apples.size()+2, 1,
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

std::vector<size_t> apple_ordering_from_metric_greedy(const std::vector<Apple> &apples,
                                                      const std::function<double(const Apple &)> &state_to_apple_distance_fn,
                                                      const std::function<double(const Apple &, const Apple &)> &apple_to_apple_distance_fn) {

    size_t first_apple = *boost::range::min_element(boost::irange<size_t>(1,apples.size()), [&](const auto a, const auto b) {
        return state_to_apple_distance_fn(apples[a]) < state_to_apple_distance_fn(apples[b]);
    });

    std::vector<bool> visited(apples.size(), false);
    visited[first_apple] = true;

    std::vector<size_t> ordering { first_apple };
    ordering.reserve(apples.size());

    while (ordering.size() < apples.size()) {

        size_t next = 0;
        double next_distance = INFINITY;

        for (const size_t i : boost::irange<size_t>(0,apples.size())) {

            double candidate_distance = apple_to_apple_distance_fn(apples[ordering.back()], apples[i]);

            if (!visited[i] && candidate_distance < next_distance) {
                next = i;
                next_distance = candidate_distance;
            }
        }

        visited[next] = true;
        ordering.push_back(next);

    }

    return ordering;


}

double ordering_heuristic_cost(const std::vector<size_t> &ordering, const std::vector<Apple> &apples,
                               const std::function<double(const Apple &)> &state_to_apple_distance_fn,
                               const std::function<double(const Apple &, const Apple &)> &apple_to_apple_distance_fn) {

    double d = state_to_apple_distance_fn(apples[ordering[0]]);

    for (const size_t i : boost::irange<size_t>(1,apples.size())) {
        d += apple_to_apple_distance_fn(apples[ordering[i-1]],apples[ordering[i]]);
    }

    return d;
}
