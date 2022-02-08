
#include <unordered_map>
#include "clustering_preselection.h"
#include "../general_utilities.h"

std::vector<size_t>
clustering::NearestKPreselection::select_around(size_t focus,
                                                const ompl::NearestNeighborsGNAT<size_t> &gnat,
                                                size_t layer) const {
    std::vector<size_t> nearby_samples;
    gnat.nearestK(focus, n, nearby_samples);
    return nearby_samples;
}

clustering::NearestKPreselection::NearestKPreselection(size_t n) : n(n) {}

std::vector<size_t> clustering::SearchRTryN::select_around(size_t focus,
                                                           const ompl::NearestNeighborsGNAT<size_t> &gnat,
                                                           size_t layer) const {
    std::vector<size_t> nearby_samples;
    gnat.nearestR(focus, radius.get(layer), nearby_samples);
    truncate(nearby_samples, n);
    return nearby_samples;
}

std::unordered_map<size_t, double>
clustering::SelectByMean::select_cluster_members(const std::vector<std::pair<size_t, double>> &candidate_members,
                                                 const size_t level) const {
    double mean = 0.0;
    for (auto&[_, distance]: candidate_members) mean += distance;
    mean /= (double) candidate_members.size();

    std::unordered_map<size_t, double> selection;

    for (auto[id, distance]: candidate_members) {
        if (distance <= mean * mean_factor) {
            selection.insert({id, distance});
        }
    }
    return selection;
}

clustering::SelectByMean::SelectByMean(double meanFactor) : mean_factor(meanFactor) {}


std::unordered_map<size_t, double> clustering::SelectByExponentialRadius::select_cluster_members(
        const std::vector<std::pair<size_t, double>> &candidate_members, const size_t level) const {

    double level_radius = radius.get(level);

    return boost::copy_range<std::unordered_map<size_t, double>>(
            candidate_members | boost::adaptors::filtered([&](const std::pair<size_t, double> &pair) {
                return pair.second < level_radius;
            })
    );

}

clustering::SelectByExponentialRadius::SelectByExponentialRadius(const clustering::LayerRadius &radius) : radius(
        radius) {}

double clustering::LayerRadius::get(size_t layer) const {
    return start_radius * std::pow(expansion_factor, layer);
}

std::unordered_map<size_t, double>
clustering::SelectAllCandidates::select_cluster_members(const std::vector<std::pair<size_t, double>> &candidate_members,
                                                        size_t level) const {
    return boost::copy_range<std::unordered_map<size_t, double> >(candidate_members);
}
