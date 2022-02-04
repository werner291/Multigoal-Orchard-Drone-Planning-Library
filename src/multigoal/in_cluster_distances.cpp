#include <boost/range/irange.hpp>
#include <random>

#include "clustering_preselection.h"
#include "gen_visitations.h"
#include "ClusterTable.h"
#include "in_cluster_distances.h"

clustering::DistanceMatrixHierarchy clustering::computeAllDistances(PointToPointPlanner &point_to_point_planner, const ClusterHierarchy& clusters) {

     assert(clusters.size() >= 1);

        std::vector<std::vector<DistanceMatrix>> distances(clusters.size()-1);

        for (size_t layer_i = 0; layer_i + 1 < clusters.size(); ++layer_i) {

            std::cout << "Computing distances for layer: " << layer_i << std::endl;

            size_t i = 0;

            distances[layer_i] = boost::copy_range<std::vector<DistanceMatrix> >(
                clusters[layer_i] | boost::adaptors::transformed([&](const Cluster& cl) {

                std::cout << "Cluster : " << (i++) << std::endl;

                return computeDistanceMatrix(point_to_point_planner, cl, clusters[layer_i+1]);
            }));
        }

        return distances;
    }

clustering::DistanceMatrix clustering::computeDistanceMatrix(
    PointToPointPlanner &point_to_point_planner,
    const Cluster& cluster,
    const std::vector<Cluster> &parent_clusters,
    double planningTimePerPair) {

    // For convenience, we copy over the parent cluster indices of the members.
    std::vector<size_t> state_ids;
    state_ids.reserve(cluster.members.size());
    for (const auto& m : cluster.members) {
        assert(m.first < parent_clusters.size());
        state_ids.push_back(m.first);
    }

    clustering::DistanceMatrix output;

    for (size_t i : boost::irange<size_t>(0,state_ids.size())) {
        output[std::make_pair(state_ids[i],state_ids[i])] = 0.0;
        for (size_t j = 0; j < i; ++j) {

            auto ptp = point_to_point_planner.planToOmplState(
                planningTimePerPair,
                parent_clusters[state_ids[i]].representative->get(),
                parent_clusters[state_ids[j]].representative->get());

            output[std::make_pair(state_ids[i],state_ids[j])] = ptp ? ptp->length() : INFINITY;
            output[std::make_pair(state_ids[j],state_ids[i])] = ptp ? ptp->length() : INFINITY;
            std::cout << "Planning from " << state_ids[i] << " to " << state_ids[i];
        }
    }

    return output;

}

clustering::StraightDistanceMetric::StraightDistanceMetric(const clustering::ClusterHierarchy &clusters) : clusters(clusters) {}

double clustering::StraightDistanceMetric::distance(const clustering::StateOrClusterRef &a,
                                        const clustering::StateOrClusterRef &b,
                                        size_t layer_id,
                                        size_t cluster_id) const  {
    const auto layer = clusters[layer_id];

    const auto repr_a = std::holds_alternative<ompl::base::ScopedStatePtr>(a) ? std::get<ompl::base::ScopedStatePtr>(a) : layer[std::get<size_t>(
            a)].representative;

    const auto repr_b = std::holds_alternative<ompl::base::ScopedStatePtr>(b) ? std::get<ompl::base::ScopedStatePtr>(b) : layer[std::get<size_t>(
            b)].representative;

    return repr_a->distance(repr_b->get());
}

clustering::InClusterPrecomputedDistanceMetricWithFallback::InClusterPrecomputedDistanceMetricWithFallback(
        const StraightDistanceMetric &fallback,
        const std::vector<std::vector<DistanceMatrix>>& distanceMatrix)
        : fallback(fallback), distanceMatrices(distanceMatrix) {}

clustering::InClusterPrecomputedDistanceMetricWithFallback::InClusterPrecomputedDistanceMetricWithFallback(
        const clustering::ClusterHierarchy& clusters,
        const std::vector<std::vector<DistanceMatrix>>& distanceMatrix)
        : fallback(clusters), distanceMatrices(distanceMatrix) {}

double clustering::InClusterPrecomputedDistanceMetricWithFallback::distance(const clustering::StateOrClusterRef &a,
                                                                const clustering::StateOrClusterRef &b,
                                                                size_t layer_id,
                                                                size_t cluster_id) const {

    if (std::holds_alternative<ompl::base::ScopedStatePtr>(a) || std::holds_alternative<ompl::base::ScopedStatePtr>(b)) {
        return fallback.distance(a, b, layer_id, cluster_id);
    } else {
        auto key = std::make_pair(std::get<size_t>(a),std::get<size_t>(b));

        auto entry = distanceMatrices[layer_id-1][cluster_id].find(key);

        assert(entry != distanceMatrices[layer_id][cluster_id].end());

        return entry->second;
    }
}