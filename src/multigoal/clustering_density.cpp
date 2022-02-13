#include <boost/range/iterator_range_core.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include "clustering_density.h"

double clustering::InverseDistanceFromCenterDensityStrategy::computeForCluster(const Cluster &new_cluster,
                                                                   const std::vector<Cluster> &previous_layer) const {
    double density = 0.0;
    for (const auto &item: new_cluster.members) {
        if (item.second > 0.0) { density += 1.0 / item.second; }
    }
    return density;
}

std::string clustering::InverseDistanceFromCenterDensityStrategy::getName() const {
    return "InverseDistanceFromCenterDensityStrategy";
}

double clustering::GoalWeightedInverseDistanceFromCenterDensityStrategy::computeForCluster(const Cluster &new_cluster,
                                                                               const std::vector<clustering::Cluster> &previous_layer) const {
    double density = 0.0;
    for (const auto &item: new_cluster.members) {
        // Ideally the clusters would count the number of represented samples.
        if (item.second > 0.0) { density += ((double) previous_layer[item.first].goals_reachable.size()) / item.second; }
    }
    return density;
}

std::string clustering::GoalWeightedInverseDistanceFromCenterDensityStrategy::getName() const {
    return "GoalWeightedInverseDistanceFromCenterDensityStrategy";
}

std::vector<double> clustering::DensityStrategy::computeForLayer(const std::vector<Cluster> &new_clusters,
                                                                 const std::vector<Cluster> &previous_layer) const {
    return boost::copy_range<std::vector<double>>(new_clusters | boost::adaptors::transformed([&](const Cluster& cl)  {
        return computeForCluster(cl, previous_layer);
    }));
}
