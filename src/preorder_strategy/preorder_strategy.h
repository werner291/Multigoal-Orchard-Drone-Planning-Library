
#ifndef NEW_PLANNERS_PREORDER_STRATEGY_H
#define NEW_PLANNERS_PREORDER_STRATEGY_H

#include <vector>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

/**
 * Abstract base class for any algorithm that takes a start position and a set of Vector3d's,
 * and proposes a series of solutions to the Euclidean TSP problem that they form. (Does not revisit start.)
 *
 * This is a class and not a functor since we might want to throw in some parameters sometime before calling it.
 */
class PreorderStrategy {

public:

    /// A small wrapper around an ordering, to be interpreted in the context of a vector of Vector3d's.
    struct Solution {
        std::vector<size_t> ordering;

        /// Compute the length of the solution, context must be provided.
        [[nodiscard]] double length(const Eigen::Vector3d &start, const std::vector<Eigen::Vector3d> &apples) const;

    };

    /**
     * Generate solutions, and return one of them at the end.
     *
     * @param start_position    A fixed starting position.
     * @param target_positions  The positions to be visited.
     * @param callback          Will be called with every proposed solution, in order. Return whether to continue.
     * @return The last solution given to the callback upon termination.
     */
    virtual Solution generate_proposals(const Eigen::Vector3d &start_position,
                                        const std::vector<Eigen::Vector3d> &target_positions,
                                        std::function<bool(std::vector<size_t>)> callback) = 0;

};

/**
 * A strategy that uses the nearest-neighbour rule, but generates different solutions by varying
 * the first target visited, in order of distance from the starting point.
 */
class KClosestNearestNeighbourOrder : public PreorderStrategy {

public:
    Solution generate_proposals(const Eigen::Vector3d &start_position,
                                const std::vector<Eigen::Vector3d> &target_positions,
                                std::function<bool(std::vector<size_t>)> callback) override;
};

/**
 * A strategy that repeatedly swaps random pairs, and accepts the result if it is better, or worse with a probability
 * that reduces exponentially over the runtime of the algorithm, like a simulated annealing process.
 */
class SimulatedAnnealingBySwapping : public PreorderStrategy {

    double cooldown_rate;
    double initial_temperature;

public:

    /**
     * @param cooldown_rate         Some value (1-epsilon) that the temperature is multiplied by every iteration.
     * @param initial_temperature   The initial probability that a worse solution may be accepted.
     */
    SimulatedAnnealingBySwapping(double cooldown_rate, double initial_temperature)
            : cooldown_rate(cooldown_rate), initial_temperature(initial_temperature) {}

    Solution
    generate_proposals(const Eigen::Vector3d &start_position, const std::vector<Eigen::Vector3d> &target_positions,
                       std::function<bool(std::vector<size_t>)> callback) override;
};

/**
 * A brute-force optimal algorithm using a branch-and-bound strategy.
 */
class BruteForceOptimal : public PreorderStrategy {

public:
    Solution
    generate_proposals(const Eigen::Vector3d &start_position, const std::vector<Eigen::Vector3d> &target_positions,
                       std::function<bool(std::vector<size_t>)> callback) override;

};

#endif //NEW_PLANNERS_PREORDER_STRATEGY_H
