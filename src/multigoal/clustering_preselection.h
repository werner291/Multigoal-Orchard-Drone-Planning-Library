#ifndef NEW_PLANNERS_CLUSTERING_PRESELECTION_H
#define NEW_PLANNERS_CLUSTERING_PRESELECTION_H

#include <vector>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

namespace clustering {

    /**
     * A pre-election strategy creates a vector of indices into a cluster hierarchy layer to which an attempt must be made to plan a path.
     *
     * This strategy is supposed to be fairly cheap, as a pre-stage to the more expensive detailed distance strategy.
     */
    class PreselectionStrategy {

    public:
        /**
         * Return a vector of indices/identifiers based on the strategy.
         *
         * @param focus The ID of the proposed cluster nucleus.
         * @param gnat  A GNAT for easy nearest-neighbour lookup based on IDs.
         * @param level The level of the cluster hierarchy.
         * @return A vector of selected IDs to plan to, preferably quite a short one.
         */
        [[nodiscard]] virtual std::vector<size_t> select_around(
                size_t focus,
                const ompl::NearestNeighborsGNAT<size_t> &gnat,
                size_t level) const = 0;

        [[nodiscard]] virtual std::string getName() const = 0;

    };

    /**
     * Simple strategy that simply selects the K nearest neighbours based on distance in the GNAT.
     */
    class NearestKPreselection : public PreselectionStrategy {

        size_t n;
    public:
        explicit NearestKPreselection(size_t n = 5);

    private:

        [[nodiscard]] std::vector<size_t>
        select_around(size_t focus, const ompl::NearestNeighborsGNAT<size_t> &gnat, size_t level) const override;

        [[nodiscard]] std::string getName() const override {
            std::stringstream ss;
            ss << "Nearest " << n;
            return ss.str();
        }

    };

    /**
     * Small utility struct that, given some parameters, will produce a radius that is exponentially proportional to
     * the given layer ID, assuming a start at 0.
     *
     * The function is very simple: start_radius * (expansion_factor^layer_number)
     */
    struct LayerRadius {
        double start_radius, expansion_factor;

        [[nodiscard]] double get(size_t layer) const;

    };

    /**
     * Preselection strategy that looks for at most K nearest neighbours within a maximum radius that exponentially
     * expands every layer of the clustering hierarchy.
     */
    class SearchRTryN : public PreselectionStrategy {

        LayerRadius radius;
        size_t n;
    public:
        SearchRTryN(double startRadius, double expansionFactor, size_t n = 5);

    public:
        [[nodiscard]] std::vector<size_t> select_around(size_t focus, const ompl::NearestNeighborsGNAT<size_t> &gnat, size_t level) const override;

        [[nodiscard]] std::string getName() const override {
            return "Search n try r";
        }

    };

    /**
     * A strategy that, given a number of identifiers and a path length to the items identified,
     * makes a selection to be used as the members of a candidate cluster.
     *
     * Note: ideally, the number of candidates is already quite small. Use PreselectionStrategy implementations as a preliminary heuristic.
     */
    class PostSelectionStrategy {

    public:
        [[nodiscard]] virtual std::unordered_map<size_t, double> select_cluster_members(const std::vector<std::pair<size_t, double>> &candidate_members, size_t level) const = 0;

        [[nodiscard]] virtual std::string getName() const = 0;
    };

    /**
     * A post-selection strategy that computes the mean distance among the candidate members,
     * and rejects anything beyond a given factor of that mean as an outlier rejection strategy.
     */
    class SelectByMean : public PostSelectionStrategy {

        double mean_factor = 1.5;

    public:
        explicit SelectByMean(double meanFactor);

        [[nodiscard]] std::unordered_map<size_t, double>
        select_cluster_members(const std::vector<std::pair<size_t, double>> &candidate_members,
                               size_t level) const override;

        [[nodiscard]] virtual std::string getName() const override {
            return "Outlier rejection by mean factor.";
        }

    };

    /**
     * Rejects anything where the actual distance is larger than a radius.
     *
     * This is useful for any kind of clusters where the actual distance is significantly larger than
     * the straight-line distance, indicating that there may be some obstacle in the way.
     */
    class SelectByExponentialRadius : public PostSelectionStrategy {

        LayerRadius radius;
    public:
        explicit SelectByExponentialRadius(const LayerRadius &radius);

    public:
        [[nodiscard]] std::unordered_map<size_t, double>
        select_cluster_members(const std::vector<std::pair<size_t, double>> &candidate_members,
                               size_t level) const override;

        [[nodiscard]] std::string getName() const override {
            return "Exponential radius.";
        }

    };

    /**
     * Simply pass through all candidates.
     */
    class SelectAllCandidates : public PostSelectionStrategy {
    public:
        [[nodiscard]] std::unordered_map<size_t, double>
        select_cluster_members(const std::vector<std::pair<size_t, double>> &candidate_members,
                               size_t level) const override;

        [[nodiscard]] virtual std::string getName() {
            return "Keep all.";
        }
    };
}


#endif //NEW_PLANNERS_CLUSTERING_PRESELECTION_H
