#ifndef NEW_PLANNERS_CLUSTERTABLE_H
#define NEW_PLANNERS_CLUSTERTABLE_H


#include <ompl/base/ScopedState.h>
#include <variant>
#include "multi_goal_planners.h"
#include "../general_utlities.h"

namespace clustering {

    struct StateAtGoal {
        size_t goal_idx;
        ompl::base::ScopedStatePtr state;
    };

    std::vector<StateAtGoal> takeInitialSamples(const GoalSet &goals,
                                                const ompl::base::SpaceInformationPtr &si,
                                                int samples_per_goal);

    struct InCluster {
        size_t member_id;
        double path_distance_from_center;
    };

    struct ClusterNeighbour {
        size_t through_member;
        size_t to_cluster;
    };

    // A coherent of collection of items from some context.
    template<typename Repr>
    struct GenericCluster {
        // The index of the representative goal sample.
        Repr representative;
        // A mapping of indices into a vector containing the items to be clustered, and a cost to reach the indicated item from the representative sample.
        // These indices either refer to goal samples directly on the lowest clustering level,
        // or to sub-clusters on higher levels of the hierarchy.
        std::map<size_t, double> members;
        // A sorted set of goal indices that are reachable through a goal sample within this cluster sub-hierarchy.
        std::set<size_t> goals_reachable;
    };

    typedef GenericCluster<ompl::base::ScopedStatePtr> Cluster;

    std::vector<Cluster> buildTrivialClusters(const std::vector<StateAtGoal> &goal_samples);

    std::vector<Cluster> create_cluster_candidates(PointToPointPlanner &point_to_point_planner,
                                                   const std::vector<StateAtGoal> &goal_samples,
                                                   double threshold,
                                                   const std::vector<Cluster> &clusters);

    std::vector<double> computeDensities(const std::vector<Cluster> &new_clusters);

    /**
     * Taking a vector of clusters as input, this algorithm will select a subset of the available clusters,
     * based on maximum local density or on whether the cluster is an outlier.
     *
     * The algorithm works by building a priority queue of each cluster based on local density, highest density first.
     *
     * It will then extract clusters one-by-one from the queue, adding to the selected set. For each cluster added
     * to the set, the neighbors will be removed from consideration (by lazy deletion), and second-order neighbours
     * will have their local density reduced accordingly (this reduces priority for consideration, which is also
     * implemented via lazy deletion).
     *
     * @param clusters The clusters to pick from.
     * @param densities The local density of each cluster.
     * @return A vector of indices into `clusters`, forming the selected subset.
     */
    std::vector<size_t> select_clusters(const std::vector<Cluster> &clusters, std::vector<double> densities);

    std::vector<std::vector<Cluster>>
    buildClusters(PointToPointPlanner &point_to_point_planner, const std::vector<StateAtGoal> &goal_samples);

    std::vector<size_t>
    visit_clusters(const std::vector<std::vector<Cluster>> &clusters,
                   const std::vector<StateAtGoal> &goal_samples,
                   const ompl::base::State *start_state);

    std::vector<size_t>
    visit_clusters_naive(const std::vector<std::vector<Cluster>> &cluster_hierarchy,
                         const std::vector<StateAtGoal> &goal_samples,
                         const ompl::base::State *start_state,
                         const ompl::base::SpaceInformation &si);


    /**
     * Recursively generate all permutations of all non-empty subsets of the provided vector of items,
     * while providing a bit of machinery to compute a left-fold of every permutation as well.
     *
     * @tparam T                The type of item in the vector of items to be permuted.
     * @tparam V                The output of the left-fold operation.
     * @param visitable         The vector of items to be permuted.
     * @param empty_value       The value of the left-fold of an empty sequence.
     * @param consider_cb       A callback, provided with the begin/end (non-inclusive) of the permuted version
     *                          of `visitable`, and the left-fold of all elements in the provided range,
     *                          EXCLUDING the last element.
     *
     *                          To return: the fold value of the full range.
     *
     * @param elements_fixed    The length of the prefix of `visitable` to keep fixed; set to zero (default option)
     *                          to generate all permutations of all non-empty subsets.
     */
    template<typename T, typename V>
    void generate_combinations
            (std::vector<T> visitable,
             const V empty_value,
             const std::function<V(std::vector<size_t>::const_iterator first,
                                   std::vector<size_t>::const_iterator last,
                                   const V &)> &consider_cb,
             size_t elements_fixed = 0) {

        // Iterate over every element beyond the range of fixed elements.
        // This intentionally includes the first element in that range.
        for (size_t swap_with = elements_fixed; swap_with < visitable.size(); ++swap_with) {
            // Swap the first element in the variable range with the pointed-to element.
            // Both indices may be euqal, in which case the swap is a no-op.
            std::swap(visitable[elements_fixed], visitable[swap_with]);

            // Call the callback with the fixed range extended by 1.
            V value = consider_cb(visitable.begin(), visitable.begin() + elements_fixed + 1, empty_value);

            // Recurse, also with the extended fixed range.
            generate_combinations(
                    visitable, value, consider_cb, elements_fixed + 1
            );

            // Undo the swap to ensure predictable behavior to bring te vector back to what it was.
            std::swap(visitable[elements_fixed], visitable[swap_with]);
        }
    }

    struct Visitation {
        size_t subcluster_index;
        std::unordered_set<size_t> goals_to_visit;

        bool operator==(const Visitation &other) const {
            return subcluster_index == other.subcluster_index && goals_to_visit == other.goals_to_visit;
        }
    };

    struct VisitationOrderSolution {
        std::vector<Visitation> visit_order;
        double cost;
        std::unordered_set<size_t> goals_visited;

        [[nodiscard]] bool is_better_than(const VisitationOrderSolution &sln) const;
    };

    /**
     * Given a collection of items, each associated with some point in space and a collection of "goal" identifiers,
     * this method will generate every possible order in which to visit the goals - with the option of skipping
     * any of them - and calculates cost and number of unique goal identifiers of each visitation order.
     *
     * Any such tour is assumed to start at a given `start_point` in space, and optionally end at a given `end_point`.
     *
     * @tparam P                 The type of the point in space with which every item is associated.
     * @tparam T                 The type of the items to be visited, or proxies thereof.
     * @param start_point        A point to start each tour from.
     * @param visitable          A vector of items to be visited.
     * @param end_point          An optional end point. If present, the cost of moving there
     *                           from the last item in the tour is added to the costs.
     * @param distance           A distance function, taking a distance between two items, a point and an item, or two points.
     *                           Parameters are std::variant<P,T>, so any combination is possible.
     * @param reachable          Returns a vector of "goal" identifiers associated with each item.
     * @param solution_callback  A callback to be called with candidate visitation orders, including a cost and number of goals visited.
     */
    template<typename P, typename T>
    void generate_visitations(
            const P &start_point,
            const std::vector<T> &visitable,
            const std::optional<P> &end_point,
            std::function<double(const std::variant<P, T> a, const std::variant<P, T> b)> distance,
            std::function<const std::set<size_t> &(const T &a)> reachable,
            const std::function<void(const VisitationOrderSolution &sol)> &solution_callback
    ) {

        // Generate an empty solution, that is either zero-cost, or goes from the start point to the end point.
        solution_callback({{}, end_point ? distance({start_point}, {*end_point}) : 0.0, {}});

        clustering::generate_combinations<size_t, VisitationOrderSolution>(
                index_vector(visitable),
                {{}, 0.0},
                [&](auto begin, auto end, const VisitationOrderSolution &cost) {

                    // Grab a copy (TODO: maybe a way to optimize by avoiding the copy? {Persistent datastructures may help})
                    VisitationOrderSolution new_cost = cost;

                    new_cost.visit_order.push_back({
                                                           *(end - 1),
                                                           {}
                                                   });

                    if (begin + 1 == end) {
                        // We're at the start of the tour, so the cost is from the starting point
                        new_cost.cost += distance({visitable[*begin]}, {start_point});
                    } else {
                        // The added cost is simply between the last and second-to-last visited goal.
                        new_cost.cost += distance({visitable[*(end - 2)]}, {visitable[*(end - 1)]});
                    }

                    // Look up which goal identifiers are reachable from here.
                    const std::set<size_t> reachable_goals = reachable(visitable[*(end - 1)]);

                    for (const size_t goal_id: reachable_goals) {
                        if (!new_cost.goals_visited.contains(goal_id)) {
                            // Mark these goals off as visited by the current tour.
                            new_cost.goals_visited.insert(goal_id);
                            new_cost.visit_order.back().goals_to_visit.insert(goal_id);
                        }
                    }

                    double partial_cost = new_cost.cost;

                    // If there is an end point, add up the cost of reaching it from the end of the current tour.
                    new_cost.cost = end_point ? (new_cost.cost + distance({visitable[*(end - 1)]}, {*end_point}))
                                              : new_cost.cost;

                    // Call the callback with the currently-generated solution.
                    solution_callback(new_cost);

                    new_cost.cost = partial_cost; // Restore old value

                    // Return the left-fold of the tour so far
                    return new_cost;
                });
    }

    /**
     * The cluster-based planner is an attempt to provide a heuristic method to solve the multi-goal planning problem.
     *
     * The planner works as follows:
     *
     * 1. k samples are taken from each goal and pooled into a single set (with a back-reference). This makes for k*n goals.
     * 2. Singleton clusters are built for each goal sample.
     * 3. Using a GNAT, paths are planned from every cluster center to all samples within a threshold Euclidean distance.
     * 4. The clusters are transposed to reveal overlap and local density.
     * 5. The degree of overlap is used to select which clusters to take to the higher level.
     * 6. Repeat from step 3 with threshold multiplied by some constant, until left with a small number of clusters, making around O(log(n)) iterations.
     *
     * Then, this hierarchical structure is used to find a good visitation order of all targets.
     *
     * Finally, a path is planned through these targets with the given visitation order and extra runtime budget.
     */
    class ClusterBasedPlanner : public MultiGoalPlanner {
    public:
        MultiGoalPlanResult
        plan(const GoalSet &goals, const ompl::base::State *start_state, PointToPointPlanner &point_to_point_planner,
             std::chrono::milliseconds time_budget) override;

        std::string getName() override;

    };

}


#endif //NEW_PLANNERS_CLUSTERTABLE_H
