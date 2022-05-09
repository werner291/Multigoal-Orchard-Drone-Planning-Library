
#ifndef NEW_PLANNERS_TRAVELING_SALESMAN_H
#define NEW_PLANNERS_TRAVELING_SALESMAN_H

#include <ortools/constraint_solver/routing.h>
#include <ortools/constraint_solver/routing_index_manager.h>
#include <ortools/constraint_solver/routing_parameters.h>
#include <boost/range/irange.hpp>
#include <utility>
#include "procedural_tree_generation.h"
#include "greatcircle.h"

class DistanceHeuristics {
public:
    [[nodiscard]] virtual std::string name() = 0;
    [[nodiscard]] virtual double first_distance(const Apple &) const = 0;
    [[nodiscard]] virtual double between_distance(const Apple &, const Apple &) const = 0;
};

class EuclideanDistanceHeuristics : public DistanceHeuristics {

    Eigen::Vector3d start_end_effector_pos;

public:
    explicit EuclideanDistanceHeuristics(Eigen::Vector3d startEndEffectorPos);

    std::string name() override;

    [[nodiscard]] double first_distance(const Apple &apple) const override;

    [[nodiscard]] double between_distance(const Apple &apple_a, const Apple &apple_b) const override;
};

class GreatcircleDistanceHeuristics : public DistanceHeuristics {

private:
    Eigen::Vector3d start_end_effector_pos;
    GreatCircleMetric gcm;

public:
    GreatcircleDistanceHeuristics(Eigen::Vector3d startEndEffectorPos, GreatCircleMetric gcm);

    std::string name() override;

    [[nodiscard]] double first_distance(const Apple &apple) const override;

    [[nodiscard]] double between_distance(const Apple &apple_a, const Apple &apple_b) const override;
};

class OrderingStrategy {
public:
    [[nodiscard]] virtual std::string name() const = 0;
    [[nodiscard]] virtual std::vector<size_t> apple_ordering(const std::vector<Apple> &apples, const DistanceHeuristics& distance) const = 0;
};

class GreedyOrderingStrategy : public OrderingStrategy {
public:
    [[nodiscard]] std::string name() const override;

    [[nodiscard]] std::vector<size_t> apple_ordering(const std::vector<Apple> &apples, const DistanceHeuristics &distance) const override;

};

class ORToolsOrderingStrategy : public OrderingStrategy {
public:
    [[nodiscard]] std::string name() const override;

    [[nodiscard]] std::vector<size_t> apple_ordering(const std::vector<Apple> &apples, const DistanceHeuristics &distance) const override;

};

double ordering_heuristic_cost(const std::vector<size_t>& ordering,
                               const std::vector<Apple>& apples,
                               const DistanceHeuristics& dh);

std::vector<size_t> tsp_open_end(const std::function<double(size_t)> &from_start, const std::function<double(size_t,size_t)> & between, size_t n);

std::vector<std::pair<size_t, size_t>> tsp_open_end_grouped(
        const std::function<double(std::pair<size_t, size_t>)> &from_start,
        const std::function<double(std::pair<size_t, size_t>, std::pair<size_t, size_t>)> &between,
        const std::vector<size_t>& sizes);

#endif //NEW_PLANNERS_TRAVELING_SALESMAN_H
