//
// Created by werner on 10-02-22.
//

#ifndef NEW_PLANNERS_AGGLOMERATIVE_CLUSTERING_H
#define NEW_PLANNERS_AGGLOMERATIVE_CLUSTERING_H

#include <variant>
#include <queue>
#include <boost/range/adaptor/transformed.hpp>

namespace agglomerative_clustering {

    /**
     * Enhanced agglomerative clustering algorithm based on:
     * https://www.researchgate.net/publication/220895309_Speeding-Up_Hierarchical_Agglomerative_Clustering_in_Presence_of_Expensive_Metrics
     */
    template<typename P>
    class AgglomerativeClustering {

        // Keeps track of the next node ID that can be assigned uniquely.
        size_t next_node_id = 0;

    public:
        // How to plan from state-to-state.
        struct DistanceResult {
            double distance;
            P midpoint;
        };

        typedef std::function<DistanceResult(const P &, const P &)> ExpensiveDistanceFn;

        ExpensiveDistanceFn distance_expensive;

        AgglomerativeClustering(const ExpensiveDistanceFn &distanceExpensive, const std::vector<P> points, int kPivots)
                : distance_expensive(distanceExpensive) {

            distance_expensive(50.0, 60.0);

            for (const P &p: points) {
                _nodes[next_node_id++].reset(new TreeNode(p, {}));
            }

            computeInitialEstimates(kPivots);

        }


        /**
         * A pair of node IDs that are potential children of a new node in the tree.
         */
        struct CandidatePair {
            // IDs of the nodes to be combined.
            std::pair<size_t, size_t> pair;
            // An upper bound on the path-planning distance between the node's representatives.
            double upper_distance_bound;
            // If available, a state from with both node representatives are reachable,
            // forming a rough center point of the hypothetical cluster to be formed.
            // May be null, in which case the `upper_distance_bound` is not a tight bound.
            std::optional<P> midpoint;

            /**
             * Whether `upper_distance_bound` is a tight bound obtained by an actual planner,
             * or merely an upper bound based on pivot distances.
             */
            bool is_tight() const {
                // midpoint is available if and only if this pair was planned for.
                return midpoint.has_value();
            }

            /**
             * Ordering, such that the priority queue puts lowest-distance pairs first.
             */
            bool operator<(const CandidatePair &other) const {
                return upper_distance_bound < other.upper_distance_bound;
            }
        };

        /**
     * A node in the clustering tree. Leaf nodes have empty children.
     */
        struct TreeNode {
            TreeNode(P representative,
                     const std::optional<std::pair<std::shared_ptr<TreeNode>, std::shared_ptr<TreeNode>>> &children)
                    : representative(representative), children(children) {}

            // A state that represents the rough center of the cluster.
            // All cluster members are supposed to be reachable from it.
            P representative;

            // Child nodes, which are empty for leaf nodes.
            std::optional<std::pair<std::shared_ptr<TreeNode>, std::shared_ptr<TreeNode>>> children;
        };

    private:
        /// Keeps track of all nodes that can be combined.
        /// Algorithm is done once this is a singleton.
        std::unordered_map<size_t, std::shared_ptr<TreeNode>> _nodes;

        /// Queue of candidate nodes to combine, ordered by distance upper bound.
        /// Note: uses lazy deletion: check candidates returned for whether that node is still open!
        std::priority_queue<CandidatePair> _candidate_pairs;

        /// A set of reference states that enable us to skip a lot of expensive, exact distance computations.
        std::vector<P> _pivots;

        std::vector<std::unordered_map<size_t, double>> pivot_distances;

    public:

        /**
         * Pulls the head of the queue, repeating and discarding pairs
         * whose nodes have been closed according to `closed_set`.
         *
         * Modifies the queue to remove the pairs returned or discarded.
         */
        CandidatePair extract_next_valid_candidate() {

            CandidatePair candidate;

            do {
                assert(!_candidate_pairs.empty());
                candidate = _candidate_pairs.top();
                _candidate_pairs.pop();
            } while (isClosed(candidate));

            return candidate;
        }
        /**
         * Runs the point-to-point planner between the representatives of the nodes,
         * and pushes a CandidatePair with exact-known distance onto the queue.
         */

        /**
         * Pick the IDs of which nodes to use as pivots, usually at the start of the algorithm.
         */

        TreeNode run() {
            // Run until the iteration method reports being done.
            while (!iterate()) {
                // nothing, the loop guard does all the work.
            }

            // Sanity check: should terminate when exactly one node left.
            assert(_nodes.size() == 1);

            // Return the sole remaining node.
            return *(_nodes.begin()->second);
        }

        bool iterate() {

            // Iteration isn't supposed to happen if we're already done.
            assert(_nodes.size() >= 2);

            // Find whichever pair is closest and merge that pair into a new node.
            merge_pair(find_closest_pair());

            std::cout << "Nodes open: " << _nodes.size() << std::endl;

            return _nodes.size() == 1;

        }

        void merge_pair(CandidatePair pair) {

            // Sanity check: we're only supposed to merge pairs once we're sure of the distance.
            assert(pair.is_tight());
            assert(!isClosed(pair));

            // Generate a unique node ID
            size_t new_node_id = next_node_id++;
            planToPivots(new_node_id, *pair.midpoint);

            // Delete the children from the open list of nodes, since they're now clustered.
            // Create a tree node, with the pair's nodes as children.
            _nodes[new_node_id].reset(new TreeNode(*pair.midpoint, {{_nodes[pair.pair.first], _nodes[pair.pair.second]}}));
            _nodes.erase(pair.pair.first);
            _nodes.erase(pair.pair.second);

            // Now, use the heuristic distance approach to create candidate pairs between the new node, and every other node.
            for (const auto&[node_id, node]: _nodes) {
                if (node_id != new_node_id)
                    // Push it onto the queue as a non-tight candidate pair.
                {
                    _candidate_pairs.push({{node_id, new_node_id},
                                           computeDistanceUpperboundThroughPivots(new_node_id, node_id), {}});
                }
            }


        }

        void planToPivots(size_t new_node_id, const P &node_representative) {// Plan to every pivot.
            for (size_t pivotI: boost::irange<size_t>(0, _pivots.size())) {
                DistanceResult dist = distance_expensive(node_representative, _pivots[pivotI]);
                pivot_distances[pivotI][new_node_id] = dist.distance;
            }
        }

        /**
         * Get an upper bound on the distance between the two given nodes.
         *
         * Precondition: pivot_distances must have entries for the two nodes.
         */
        double computeDistanceUpperboundThroughPivots(size_t new_node_id, size_t node_id) {
            double distance = std::numeric_limits<double>::infinity();
            for (size_t pivot_i: boost::irange<size_t>(0, _pivots.size())) {

                assert(pivot_distances[pivot_i].count(new_node_id) != 0);
                assert(pivot_distances[pivot_i].count(node_id) != 0);

                distance = std::min(distance, pivot_distances[pivot_i][new_node_id] + pivot_distances[pivot_i][node_id]);
            }
            return distance;
        }

        bool isClosed(
                const CandidatePair &candidate) const {
            return _nodes.count(candidate.pair.first) == 0 || _nodes.count(candidate.pair.second) == 0;
        }

        /**
         * Extracts the CandidatePair with the lowest distance, whose distance must be exact.
         *
         * Exact distances are computed as necessary.
         */
        CandidatePair find_closest_pair() {

            // Pull from the queue until we get a non-lazy-deleted candidate pair.
            auto candidate = extract_next_valid_candidate();

            // If the distance estimate is not exact, we cannot assume that this is truly the closest pair.
            while (!candidate.is_tight()) {

                // If not, tighten the bound and put it back in the queue. If it's truly the closest pair,
                // we should get it right back. Otherwise, we'll want to look at other pairs first.
                tighten_and_enqueue(candidate.pair.first, candidate.pair.second);

                // Pull the next candidate and repeat.
                candidate = extract_next_valid_candidate();
            }

            // Return what we now know is truly the closest pair.
            return candidate;
        }

        void tighten_and_enqueue(size_t a, size_t b) {
            assert(_nodes.count(a) != 0);
            assert(_nodes.count(b) != 0);
            double ra = _nodes.find(a)->second->representative;
            double rb = _nodes.find(b)->second->representative;
            DistanceResult dist = distance_expensive(ra, rb);

            CandidatePair candidate {{a, b}, dist.distance, {dist.midpoint}};

            _candidate_pairs.emplace(candidate);
        }

        void pick_pivots(size_t n) {
            std::vector<size_t> choices;
            choices.reserve(n);

            ompl::RNG rng;

            for (size_t i: boost::irange<size_t>(0, n)) {

                size_t choice = rng.uniformInt(0, _nodes.size() - choices.size()-1);

                for (size_t j: choices) {
                    if (choice >= j) {
                        choice += 1;
                    }
                }

                choices.push_back(choice);
            }

            _pivots = boost::copy_range<std::vector<P>>(choices | boost::adaptors::transformed([&](size_t node_id) {
                assert(_nodes.count(node_id) == 1);
                return _nodes[node_id]->representative;
            }));

            pivot_distances.resize(_pivots.size());
        }

        void computeInitialEstimates(size_t k_pivots) {

            pick_pivots(k_pivots);


            for (const auto&[node_id, node]: _nodes) {
                planToPivots(node_id, node->representative);
            }

            for (const auto&[node_i, _]: _nodes) {
                for (const auto&[node_j, _]: _nodes) {
                    if (node_j >= node_i) continue;
                    _candidate_pairs.push({{node_i, node_j}, computeDistanceUpperboundThroughPivots(node_i, node_j), {}});
                }
            }


        }


    };

}

#endif //NEW_PLANNERS_AGGLOMERATIVE_CLUSTERING_H
