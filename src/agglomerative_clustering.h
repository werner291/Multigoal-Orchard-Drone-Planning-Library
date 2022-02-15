//
// Created by werner on 10-02-22.
//

#ifndef NEW_PLANNERS_AGGLOMERATIVE_CLUSTERING_H
#define NEW_PLANNERS_AGGLOMERATIVE_CLUSTERING_H

#include <variant>
#include <ompl/base/ScopedState.h>
#include "multigoal/PointToPointPlanner.h"

namespace agglomerative_clustering {

    /**
     * Enhanced agglomerative clustering algorithm based on:
     * https://www.researchgate.net/publication/220895309_Speeding-Up_Hierarchical_Agglomerative_Clustering_in_Presence_of_Expensive_Metrics
     */
    template<typename P>
    class AgglomerativeClustering {

        // Keeps track of the next node ID that can be assigned uniquely.
        size_t next_node_id;

        // How to plan from state-to-state.
        struct DistanceResult {
            double distance;
            P midpoint;
        };
        std::function<DistanceResult(const P&, const P&)> distance_expensive;

    public:
        AgglomerativeClustering(const std::function<double(const P &, const P &)> &distanceExpensive):
            distance_expensive(distanceExpensive) {}

    public:
        /**
         * A node in the clustering tree. Leaf nodes have empty children.
         */
        struct TreeNode {
            // A state that represents the rough center of the cluster.
            // All cluster members are supposed to be reachable from it.
            P representative;

            TreeNode(const P &representative,
                     const std::optional<std::pair<std::shared_ptr<TreeNode>, std::shared_ptr<TreeNode>>> &children);

            // Child nodes, which are empty for leaf nodes.
            std::optional<std::pair<std::shared_ptr<TreeNode>,std::shared_ptr<TreeNode>>> children;
        };

        /**
         * A pair of node IDs that are potential children of a new node in the tree.
         */
        struct CandidatePair {
            // IDs of the nodes to be combined.
            std::pair<size_t,size_t> pair;
            // An upper bound on the path-planning distance between the node's representatives.
            double upper_distance_bound;
            // If available, a state from with both node representatives are reachable,
            // forming a rough center point of the hypothetical cluster to be formed.
            // May be null, in which case the `upper_distance_bound` is not a tight bound.
            P midpoint;

            /**
             * Whether `upper_distance_bound` is a tight bound obtained by an actual planner,
             * or merely an upper bound based on pivot distances.
             */
            bool is_tight() {
                // midpoint is available if and only if this pair was planned for.
                return midpoint->get() != nullptr;
            }

            /**
             * Ordering, such that the priority queue puts lowest-distance pairs first.
             */
            bool operator<(const CandidatePair& other) {
                return upper_distance_bound < other.upper_distance_bound;
            }
        };

    private:
        /// Keeps track of all nodes that can be combined.
        /// Algorithm is done once this is a singleton.
        std::unordered_map<size_t,std::shared_ptr<TreeNode>> _nodes;

        /// Queue of candidate nodes to combine, ordered by distance upper bound.
        /// Note: uses lazy deletion: check candidates returned for whether that node is still open!
        std::priority_queue<CandidatePair> _candidate_pairs;

        /// A set of reference states that enable us to skip a lot of expensive, exact distance computations.
        std::vector<ompl::base::ScopedStatePtr> _pivots;

        std::vector<std::unordered_map<size_t, double>> pivot_distances;

    public:

        /**
         * Pulls the head of the queue, repeating and discarding pairs
         * whose nodes have been closed according to `closed_set`.
         *
         * Modifies the queue to remove the pairs returned or discarded.
         */
        CandidatePair extract_next_valid_candidate();

        /**
         * Runs the point-to-point planner between the representatives of the nodes,
         * and pushes a CandidatePair with exact-known distance onto the queue.
         */
        void tighten_and_enqueue(size_t a, size_t b);

        /**
         * Extracts the CandidatePair with the lowest distance, whose distance must be exact.
         *
         * Exact distances are computed as necessary.
         */
        CandidatePair find_closest_pair();

        /**
         * Pick the IDs of which nodes to use as pivots, usually at the start of the algorithm.
         */
        void pick_pivots(size_t n);

        void compute_pivot_distance_matrix();

        /**
         * Populates the queue of candidate_pairs by computing the k*n distance matrix,
         * where `n` is the number of open pairs and `k` the number of pivots.
         *
         * @param k_pivots Number of pivots.
         */
        void computeInitialEstimates(size_t k_pivots);

        void merge_pair(CandidatePair pair);

        bool iterate();

        TreeNode run() {
            while (!iterate()) {
                // nothing, the loop guard does all the work.
            }

            return _nodes.begin()->second;
        }

        bool isNotClosed(const CandidatePair &candidate) const;

        void insert_and_plan_to_pivots(CandidatePair &pair, size_t new_node_id);
    };

}

#endif //NEW_PLANNERS_AGGLOMERATIVE_CLUSTERING_H
