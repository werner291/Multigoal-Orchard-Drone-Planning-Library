
#ifndef NEW_PLANNERS_TRAVELING_SALESMAN_H
#define NEW_PLANNERS_TRAVELING_SALESMAN_H

#include <ortools/constraint_solver/routing.h>
#include <ortools/constraint_solver/routing_index_manager.h>
#include <ortools/constraint_solver/routing_parameters.h>
#include <boost/range/irange.hpp>
#include "procedural_tree_generation.h"
#include "greatcircle.h"

std::vector<size_t>
apple_ordering_from_metric_greedy(const std::vector<Apple> &apples,
                           const std::function<double(const Apple&)>& state_to_apple_distance_fn,
                           const std::function<double(const Apple&, const Apple&)>& apple_to_apple_distance_fn);

std::vector<size_t>
apple_ordering_from_metric_ortools(const std::vector<Apple> &apples,
                                   const std::function<double(const Apple&)>& state_to_apple_distance_fn,
                                   const std::function<double(const Apple&, const Apple&)>& apple_to_apple_distance_fn);

double ordering_heuristic_cost(const std::vector<size_t>& ordering,
                               const std::vector<Apple>& apples,
                               const std::function<double(const Apple&)>& state_to_apple_distance_fn,
                               const std::function<double(const Apple&, const Apple&)>& apple_to_apple_distance_fn);

#endif //NEW_PLANNERS_TRAVELING_SALESMAN_H
