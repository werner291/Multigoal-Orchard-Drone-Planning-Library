#ifndef NEW_PLANNERS_CLUSTERING_PRESELECTION_H
#define NEW_PLANNERS_CLUSTERING_PRESELECTION_H

#include <vector>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

namespace clustering {

    class PreselectionStrategy {

    public:
        virtual std::vector<size_t> select_around(
                size_t focus,
                const ompl::NearestNeighborsGNAT<size_t> &gnat,
                size_t level) const = 0;

    };

    class NearestKPreselection : public PreselectionStrategy {

        size_t n;
    public:
        NearestKPreselection(size_t n = 5);

    private:

        std::vector<size_t>
        select_around(size_t focus, const ompl::NearestNeighborsGNAT<size_t> &gnat, size_t level) const override;

    };

    class SearchRTryN : public PreselectionStrategy {

        double start_radius, expansion_factor;
        size_t n;
    public:
        SearchRTryN(double startRadius, double expansionFactor, size_t n = 5);

    public:
        std::vector<size_t>
        select_around(size_t focus, const ompl::NearestNeighborsGNAT<size_t> &gnat, size_t level) const override;

    };

}

#endif //NEW_PLANNERS_CLUSTERING_PRESELECTION_H
