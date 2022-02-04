#ifndef NEW_PLANNERS_IN_CLUSTER_DISTANCES_H
#define NEW_PLANNERS_IN_CLUSTER_DISTANCES_H

#include "clustering_types.h"
#include "PointToPointPlanner.h"

namespace clustering {

    /**
     *  A function/strategy to compute some kind of distance between two states (and their representatives),
     *  optionally involving an arbitrary state from some outside context.
     */
    class ClusterDistanceFn {
    public:
        virtual double distance(const StateOrClusterRef &a,
                                const StateOrClusterRef &b,
                                size_t layer_id,
                                size_t cluster_id) const = 0;
    };

    /**
     * In this case: simply compute the straight-line distance between representatives/states.
     */
    class StraightDistanceMetric : public ClusterDistanceFn {

    public:
        const std::vector<std::vector<Cluster>> &clusters;

        StraightDistanceMetric(const std::vector<std::vector<Cluster>> &clusters);

        virtual double distance(const StateOrClusterRef &a,
                                const StateOrClusterRef &b,
                                size_t layer_id,
                                size_t cluster_id) const override;
    };

    typedef std::map<std::pair<size_t, size_t>, double> DistanceMatrix;
    typedef std::vector<std::vector<DistanceMatrix>> DistanceMatrixHierarchy;

    DistanceMatrixHierarchy computeAllDistances(PointToPointPlanner &point_to_point_planner,
                                                const std::vector<std::vector<Cluster>> &clusters);

    DistanceMatrix computeDistanceMatrix(PointToPointPlanner &point_to_point_planner,
                                         const Cluster &forCluster,
                                         const std::vector<Cluster> &clusters,
                                         double planningTimePerPair = 0.2);

    /*
     * Distance metric used is the actual, planned distance within a cluster.
     *
     * If an outside state is presented, the straight-line distance is used as a fallback.
     */
    class InClusterPrecomputedDistanceMetricWithFallback : public ClusterDistanceFn {
        StraightDistanceMetric fallback;
        const DistanceMatrixHierarchy &distanceMatrices;
    public:
        InClusterPrecomputedDistanceMetricWithFallback(
                const StraightDistanceMetric &fallback,
                const std::vector<std::vector<DistanceMatrix>> &distanceMatrix);

        InClusterPrecomputedDistanceMetricWithFallback(
                const std::vector<std::vector<Cluster>> &clusters,
                const std::vector<std::vector<DistanceMatrix>> &distanceMatrix);

        virtual double distance(const StateOrClusterRef &a,
                                const StateOrClusterRef &b,
                                size_t layer_id,
                                size_t cluster_id) const override;
    };


}

#endif //NEW_PLANNERS_IN_CLUSTER_DISTANCES_H
