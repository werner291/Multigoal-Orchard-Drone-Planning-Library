
#ifndef NEW_PLANNERS_CLUSTERING_TYPES_H
#define NEW_PLANNERS_CLUSTERING_TYPES_H

#include <map>
#include <set>
#include <cstddef> // For size_t
#include <variant>
#include <ompl/base/ScopedState.h>

namespace clustering {

    /**
     * A robot state associated with a goal index.
     */
    struct StateAtGoal {
        size_t goal_idx;
        ompl::base::ScopedStatePtr state;
    };

    /**
     * Reperesents a cluster within a clustering-based visitation order generator.
     */
    struct Cluster {
        // The representative state
        ompl::base::ScopedStatePtr representative;
        // A mapping of indices into a vector containing the items to be clustered, and a cost to reach the indicated item from the representative sample.
        // These indices either refer to goal samples directly on the lowest clustering level,
        // or to sub-clusters on higher levels of the hierarchy.
        std::map<size_t, double> members;
        // A sorted set of goal indices that are reachable through a goal sample within this cluster sub-hierarchy.
        std::set<size_t> goals_reachable;

        /**
         * Mark a given goal as reachable, performing some internal bookkeeping.
         */
        void add_reachable(const std::vector <Cluster> &parent_layer, size_t which, double distance);

        /**
         * Build a cluster with no (non-trivial) members around a given parent cluster.
         */
        static Cluster new_around(const std::vector <Cluster> &parent_layer, size_t parent);
    };

    /**
     * Either a shared, scoped pointer to a state, or a numeric ID referring
     * to a cluster on some level of a hierarchy.
     */
    typedef std::variant<ompl::base::ScopedStatePtr, size_t> StateOrClusterRef;

    typedef std::vector<std::vector<Cluster>> ClusterHierarchy;


    std::unordered_map<size_t, double>
    select_cluster_members(const std::vector<std::pair<size_t, double>> &candidate_members, const size_t level);
}

#endif //NEW_PLANNERS_CLUSTERING_TYPES_H
