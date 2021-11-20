

#include "preorder_strategy.h"
#include "../multigoal/goals_gnat.h"
#include "../experiment_utils.h"
#include <vector>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <fstream>

PreorderStrategy::Solution KClosestNearestNeighbourOrder::generate_proposals(const Eigen::Vector3d &start_position,
                                                                             const std::vector<Eigen::Vector3d> &target_positions,
                                                                             std::function<bool(
                                                                                     std::vector<size_t>)> callback) {
    assert(!target_positions.empty());

    // Build a GNAT to sort the targets by distance from the start position.
    std::vector<GNATNode> nearestKToStart;
    buildGoalGNAT<Eigen::Vector3d>(target_positions, [](const Eigen::Vector3d &tgt) { return tgt; })
            .nearestK({SIZE_MAX, start_position}, target_positions.size(), nearestKToStart);

    // We return the last solution, either after running out of candidates or the callback returning false.
    Solution solution;

    // We try to build solutions starting at sidderent targets in order of distance from start point.
    for (const GNATNode &first_point: nearestKToStart) {

        assert(first_point.goal != SIZE_MAX); // Make sure we don't get the dummy lookup value for some weird reason.

        // Build an index list of the targets. These are indices into `target_positions`.
        std::vector<size_t> points_in_order{first_point.goal};

        // We build a GNAT every iteration since it is consumed by the process.
        auto nn = buildGoalGNAT<Eigen::Vector3d>(target_positions, [](const Eigen::Vector3d &tgt) { return tgt; });

        // Remove the first point, since we're starting there and don't need to revisit it.
        nn.remove(first_point);

        // Extract points one-by-one and push them into the order.
        while (nn.size() > 0) {
            auto next = nn.nearest({
                                           points_in_order.back(),
                                           target_positions[points_in_order.back()]
                                   });
            nn.remove(next);
            points_in_order.push_back(next.goal);
        }

        // Store the solution in case we need to return it at the end.
        solution.ordering = points_in_order;

        // Inform the callback, and return if it returns false.
        if (!callback(points_in_order)) break;
    }

    // Return the last solution we built.
    return solution;

}

double
PreorderStrategy::Solution::length(const Eigen::Vector3d &start, const std::vector<Eigen::Vector3d> &apples) const {
    assert(!apples.empty());
    double length = (apples[ordering[0]] - start).norm();
    for (size_t idx = 0; idx + 1 < ordering.size(); ++idx) {
        length += (apples[ordering[idx]] - apples[ordering[idx + 1]]).norm();
    }
    return length;
}

PreorderStrategy::Solution SimulatedAnnealingBySwapping::generate_proposals(const Eigen::Vector3d &start_position,
                                                                            const std::vector<Eigen::Vector3d> &target_positions,
                                                                            std::function<bool(
                                                                                    std::vector<size_t>)> callback) {

    double temperature = initial_temperature;

    // Start from the first nearest-neighbors solution.
    Solution current_solution = KClosestNearestNeighbourOrder().generate_proposals(start_position, target_positions,
                                                                                   [&](const std::vector<size_t> &solution) {
                                                                                       return false; // accept the first one.
                                                                                   });

    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()

    // Keep going while the callback returns true.
    while (callback(current_solution.ordering)) {

        // Pick two indices uniformly at random (no replacement, ordered)
        auto pair = generateIndexPairNoReplacement(gen, current_solution.ordering.size());

        // Copy the current solution and swap (This is expensive, can easily be optimized by triangle inequality if need be)
        Solution new_solution = current_solution;
        std::swap(new_solution.ordering[pair.first], new_solution.ordering[pair.second]);

        // Accept if better, or worse and temperature test passes.
        if (std::generate_canonical<double, 10>(gen) < temperature ||
            new_solution.length(start_position, target_positions) <
            current_solution.length(start_position, target_positions)) {
            current_solution = std::move(new_solution);
        }

        // Reduce the temperature over time.
        temperature *= cooldown_rate;
    }

    // Return whatever remains post-termination.
    return current_solution;
}

/**
 * Recursively visits all permutations of the target positions and finds the optimal visitation order
 * by brute force. Groups of permutations may be rejected early if the cost of some prefix is higher
 * than the best-known cost.
 *
 * @param depth             How many positions of target_positions are fixed, from the left/
 * @param start_position    Fixed starting position, for the first segment of the solution.
 * @param target_positions  A vector of positions to be visited (paired with the index in the original vector).
 *                          Will be mutated starting from `depth` index.
 * @param costSoFar         The cost of visiting all targets up to (not including) `target_positions[depth]`.
 * @param bestCost          The cost fo the best-known solution.
 * @param bestSolution      The best-known visitation order.
 */
void heap_recurse(size_t depth,
                  const Eigen::Vector3d &start_position,
                  std::vector<std::pair<size_t, Eigen::Vector3d>> &target_positions,
                  const double costSoFar,
                  double &bestCost,
                  std::vector<std::pair<size_t, Eigen::Vector3d>> &bestSolution) {

    // Cost can only go up as we go deeper, so no point in doing so
    // if the prefix is more expensive than the best-known solution.
    if (costSoFar > bestCost) return;

    if (depth == target_positions.size()) {

        // The order of the whole array is fixed, the cost of this order is thus fully known.
        // Record it if it is better than the best-known.
        if (costSoFar < bestCost) {
            bestCost = costSoFar;
            bestSolution = target_positions;
        }

    } else {

        // The point from which we travel to the next candidate goal.
        Eigen::Vector3d previous = depth == 0 ? start_position : target_positions[depth - 1].second;

        // Go through all unvisited goals as possible candidates as the next to visit.
        for (size_t i = depth; depth < target_positions.size(); ++i) {

            // Swap it into the next position.
            std::swap(target_positions[i], target_positions[depth]);

            // Recurse, with that position now fixed.
            heap_recurse(depth + 1, start_position, target_positions,
                         costSoFar + (previous - target_positions[depth].second).norm(), bestCost, bestSolution);

            // Undo the swap to make sure the candidate selection process is not disturbed.
            std::swap(target_positions[i], target_positions[depth]);

        }
    }
}

BranchAndBoundOptimal::Solution BranchAndBoundOptimal::generate_proposals(
        const Eigen::Vector3d &start_position,
        const std::vector<Eigen::Vector3d> &target_positions,
        std::function<bool(std::vector<size_t>)> callback) {

    // Start with the nearest-neighbours ordering for a good first bound, and maybe a coincidentally-optimal solution.
    Solution best_so_far;
    KClosestNearestNeighbourOrder()
            .generate_proposals(start_position,
                                target_positions,
                                [&](std::vector<size_t> solution) {
                                    best_so_far = {std::move(solution)};
                                    return false; /* accept the first one. */ });

    // Pair up the target positions with their index, so that the index order can be reconstructed after permuting.
    std::vector<std::pair<size_t, Eigen::Vector3d>> target_positions_with_index;
    for (size_t i = 0; i < target_positions.size(); ++i) {
        target_positions_with_index.emplace_back(i, target_positions[i]);
    }

    // Same here, but with the nearest-neighbors solution.
    std::vector<std::pair<size_t, Eigen::Vector3d>> best_with_index;
    for (unsigned long &i: best_so_far.ordering) {
        best_with_index.emplace_back(i, target_positions[i]);
    }

    // Get the cost of the nearest-neighbors solution as a first upper bound.
    double best_cost = best_so_far.length(start_position, target_positions);

    // Start the branch-and-bound procedure.
    heap_recurse(
            0, start_position, target_positions_with_index, 0.0, best_cost, best_with_index
    );

    // Strip out the position information, keeping the indices, to comply with the return value contract.
    std::vector<size_t> solution_indices;
    for (const auto &item: best_with_index) solution_indices.push_back(item.first);

    // Return the solution.
    return {
            solution_indices
    };
}