
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

clustering::SearchRTryN::SearchRTryN(double startRadius, double expansionFactor, size_t n) :
        start_radius(startRadius),
        expansion_factor(expansionFactor), n(n) {}

std::vector<size_t> clustering::SearchRTryN::select_around(size_t focus,
                                                           const ompl::NearestNeighborsGNAT<size_t> &gnat,
                                                           size_t layer) const {
    std::vector<size_t> nearby_samples;
    gnat.nearestR(focus, start_radius * std::pow(expansion_factor, layer), nearby_samples);
    truncate(nearby_samples, n);
    return nearby_samples;
}
