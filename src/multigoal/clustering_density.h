#ifndef NEW_PLANNERS_CLUSTERING_DENSITY_H
#define NEW_PLANNERS_CLUSTERING_DENSITY_H

#include "clustering_types.h"

namespace clustering {
    class DensityStrategy {

    public:
        [[nodiscard]] virtual double computeForCluster(const Cluster& new_cluster, const std::vector<Cluster> &previous_layer) const = 0;

        [[nodiscard]] virtual std::vector<double> computeForLayer(const std::vector<Cluster> &new_clusters, const std::vector<Cluster> &previous_layer) const;

        [[nodiscard]] virtual std::string getName() const = 0;
    };

    class InverseDistanceFromCenterDensityStrategy : public DensityStrategy {
        [[nodiscard]] double computeForCluster(const Cluster &new_cluster,
                                 const std::vector<Cluster> &previous_layer) const override;

    public:
        [[nodiscard]] std::string getName() const override;


    };

    class GoalWeightedInverseDistanceFromCenterDensityStrategy : public DensityStrategy {
        [[nodiscard]] double computeForCluster(const Cluster &new_cluster,
                                 const std::vector<Cluster> &previous_layer) const override;

    public:
        std::string getName() const override;

    };
}

#endif //NEW_PLANNERS_CLUSTERING_DENSITY_H
