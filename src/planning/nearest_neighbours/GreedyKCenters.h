/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Mark Moll */
/* Somewhat strongly modified by  Werner Kroneman to remove OMPL dependencies and for documentation/understanding purposes */
/* Original code at https://github.com/ompl/ompl/blob/af4d6d625e15a4ed6d85255c718ce26bcd79e4cc/src/ompl/datastructures/GreedyKCenters.h#L50 */

#ifndef OMPL_DATASTRUCTURES_GREEDY_K_CENTERS_
#define OMPL_DATASTRUCTURES_GREEDY_K_CENTERS_

#include <cassert>
#include <functional>
#include <optional>

#include "../RandomNumberGenerator.h"

/**
 * A set of hooks that can be used to observe the behavior of the greedy k centers algorithm.
 */
struct GreedyKCentersHooks {
    // Called when a new center is added to the set of centers.
    std::function<void(size_t)> centerAdded = [](size_t) {
    };
};

/**
 * Greedy K centers algorithm, as based on the implementation in OMPL.
 *
 * Given a set of data points, this function picks a set of k centers such that, when each data point is assigned
 * to the nearest center, the maximum distance between a data point and its assigned center is roughly minimized.
 *
 * It works as follows: the first center is picked randomly, and then the next center is the data point that is
 * furthest away from the centers computed so far. This is repeated until k centers have been computed.
 *
 * The internal structure of the loop contains a few tricks that are a bit hard to understand at first (as there were in the original),
 * but to the best of my understanding, they *do* work.
 *
 * @param data      The data points to cluster.
 * @param k         The number of centers to compute.
 * @param distFun   A function that computes the distance between two data points.
 * @param rng       A random number generator.
 * @param hooks     A set of hooks that can be used to observe the behavior of the algorithm.
 *
 * @returns     A vector of indices of the computed centers. If fewer than k data points are available, this vector may be smaller than k.
 */
template<typename T>
std::vector<size_t> greedy_k_centers(
    const std::vector<T> &data,
    unsigned int k,
    const std::function<double(const T &, const T &)> &distFun,
    random_numbers::RandomNumberGenerator &rng,
    const std::optional<GreedyKCentersHooks> hooks = std::nullopt
) {
    // A vector of indices of the computed centers; indices are into the data vector
    std::vector<size_t> centers;
    centers.reserve(k);

    // A matrix to store the distances between each data point and the closest center.
    // At this point, there are no centers yet, so all distances are infinite.
    std::vector<double> minDist(data.size(), std::numeric_limits<double>::infinity());

    // First center is picked randomly
    centers.push_back(rng.uniformInteger(0, data.size() - 1));

    if (hooks) hooks->centerAdded(centers.back());

    // For the other k - 1 centers...
    while (centers.size() < k && centers.size() < data.size()) {
        // Find the index of the data point that is furthest away from the centers computed so far
        unsigned best_index = 0;
        double best_index_distance = -std::numeric_limits<double>::infinity();

        // Go over all data points, and for each:
        // - Update the minimum distance between the data point and the closest center.
        // - Find whichever point is now furthest away from the centers computed so far, and pick it as the next center.
        //
        // Note that this loop should be possible to split into two loops, but it's combined sinc eit was so in the original.
        for (unsigned j = 0; j < data.size(); ++j) {
            // Invariant: at the top of this loop, minDist[j] contains the minimum distance between `data[j]` and any of the centers, except for the last one.
            assert(std::all_of(centers.begin(), centers.end() - 1, [&](size_t center_index) {
                return minDist[j] <= distFun(data[j], data[center_index]);
                }));

            // Find the distance between the current data point and the last center.
            // That is, the minimum distance between `data[j]` and any of `centers`.
            double distance_from_last_center = distFun(data[j], data[centers.back()]);

            if (distance_from_last_center < minDist[j]) {
                // The new candidate center is closer to the last center than the previous closest center; store the new distance.
                minDist[j] = distance_from_last_center;
            }

            // At this point in the loop, `minDist[j]` contains the minimum distance between `data[j]` and any of the centers.
            // The loop can be split here, but it's not done so in the original.

            if (minDist[j] > best_index_distance) {
                // Point j is further away from the closest center than points 0..j-1 are from their closest center.
                best_index = j;
                best_index_distance = minDist[j];
            }
        }

        // At this point, `best_index` contains the index of the data point that is furthest away from the centers computed so far.
        // That is: there is no point that is further away from the centers than `data[best_index]`.
        assert(std::all_of(data.begin(), data.end(), [&](const T &point) {
            return std::any_of(centers.begin(), centers.end(), [&](size_t center_index) {
                return distFun(point, data[center_index]) <= distFun(data[best_index], data[center_index]);
                });
            }));

        // We can't find a point that is any meaningful distance away from its closest center. Stop.
        if (best_index_distance < std::numeric_limits<double>::epsilon())
            break;

        // Add the new center to the list of centers.
        centers.push_back(best_index);

        // Notify the hooks that a new center has been added.
        if (hooks) hooks->centerAdded(centers.back());
    }

    return centers;
}

#endif
