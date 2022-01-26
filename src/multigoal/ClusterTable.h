#ifndef NEW_PLANNERS_CLUSTERTABLE_H
#define NEW_PLANNERS_CLUSTERTABLE_H

#include <ompl/base/ScopedState.h>
#include <variant>
#include <boost/range/adaptors.hpp>
#include <boost/range/adaptor/transformed.hpp>

#include "multi_goal_planners.h"
#include "../general_utilities.h"

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

    struct Cluster {
        // The index of the representative goal sample.
        ompl::base::ScopedStatePtr representative;
        // A mapping of indices into a vector containing the items to be clustered, and a cost to reach the indicated item from the representative sample.
        // These indices either refer to goal samples directly on the lowest clustering level,
        // or to sub-clusters on higher levels of the hierarchy.
        std::map<size_t, double> members;
        // A sorted set of goal indices that are reachable through a goal sample within this cluster sub-hierarchy.
        std::set<size_t> goals_reachable;

        void add_reachable(const std::vector<Cluster> &parent_layer, size_t which, double distance);

        static Cluster new_around(const std::vector<Cluster> &parent_layer, size_t parent);
    };

    std::vector<Cluster> buildTrivialClusters(const std::vector<StateAtGoal> &goal_samples);

    const size_t DEFAULT_CLUSTER_SIZE = 5;

    std::vector<Cluster> create_cluster_candidates(PointToPointPlanner &point_to_point_planner,
                                                   const std::vector<StateAtGoal> &goal_samples,
                                                   double threshold,
                                                   const std::vector<Cluster> &clusters,
                                                   const size_t max_cluster_size = DEFAULT_CLUSTER_SIZE,
                                                   double time_per_ptp = 0.2);

    std::vector<double> computeDensities(const std::vector<Cluster> &new_clusters);

    typedef std::map<std::pair<size_t,size_t>,double> DistanceMatrix;

//    DistanceMatrix computeDistanceMatrix(PointToPointPlanner &point_to_point_planner,
//                                         const std::vector<Cluster> &clusters);

    DistanceMatrix computeDistanceMatrix(PointToPointPlanner &point_to_point_planner,
                                         const Cluster &forCluster,
                                         const std::vector<Cluster> &clusters,
                                         double planningTimePerPair = 0.2);

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

    typedef std::vector<std::vector<Cluster>> ClusterHierarchy;

    ClusterHierarchy buildClusters(PointToPointPlanner &point_to_point_planner,
                                   const std::vector<StateAtGoal> &goal_samples);

    std::vector<std::vector<DistanceMatrix>>
    computeAllDistances(PointToPointPlanner &point_to_point_planner, const ClusterHierarchy &clusters);

    std::vector<size_t>
    visit_clusters(const std::vector<std::vector<Cluster>> &clusters,
                   const std::vector<StateAtGoal> &goal_samples,
                   const ompl::base::State *start_state);

    std::vector<size_t>
    visit_clusters_naive(const std::vector<std::vector<Cluster>> &cluster_hierarchy,
                         const std::vector<StateAtGoal> &goal_samples,
                         const ompl::base::State *start_state,
                         const ompl::base::SpaceInformation &si);


    struct Visitation {
        size_t subcluster_index;
        std::set<size_t> goals_to_visit;

        bool operator==(const Visitation &other) const {
            return subcluster_index == other.subcluster_index && goals_to_visit == other.goals_to_visit;
        }
    };

    struct VisitationOrderSolution {
        std::vector<Visitation> visit_order;
        double cost;
        std::set<size_t> goals_visited;

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
            const std::set<size_t> &goals_to_visit,
            const std::optional<P> &end_point,
            std::function<double(const std::variant<P, T> a, const std::variant<P, T> b)> distance,
            std::function<const std::set<size_t> &(const T &a)> reachable,
            const std::function<void(const VisitationOrderSolution &sol)> &solution_callback
    ) {

        // Generate an empty solution, that is either zero-cost, or goes from the start point to the end point.
        solution_callback({{}, end_point ? distance({start_point}, {*end_point}) : 0.0, {}});

        generate_combinations<size_t, VisitationOrderSolution>(
                index_vector(visitable),
                {{}, 0.0},
                [&](auto begin, auto end, const VisitationOrderSolution &cost) {

                    // Grab a copy (TODO: maybe a way to optimize by avoiding the copy? {Persistent datastructures may help})
                    VisitationOrderSolution new_cost = cost;

                    new_cost.visit_order.push_back({*(end - 1),{}});

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
                        if (new_cost.goals_visited.count(goal_id) == 0 && goals_to_visit.count(goal_id) > 0) {
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

    using StateOrClusterRef = std::variant<ompl::base::ScopedStatePtr, size_t> ;

    class ClusterDistanceFn {
    public:
        virtual double distance(const StateOrClusterRef& a,
                                const StateOrClusterRef& b,
                                size_t layer_id,
                                size_t cluster_id) const = 0;
    };


    class StraightDistanceMetric : public ClusterDistanceFn {

    public:
        const ClusterHierarchy& clusters;
        StraightDistanceMetric(const ClusterHierarchy &clusters);

        virtual double distance(const StateOrClusterRef& a,
                              const StateOrClusterRef& b,
                              size_t layer_id,
                              size_t cluster_id) const override;
    };

    class InClusterPrecomputedDistanceMetricWithFallback : public ClusterDistanceFn {
        StraightDistanceMetric fallback;
        const std::vector<std::vector<DistanceMatrix>>& distanceMatrices;
    public:
        InClusterPrecomputedDistanceMetricWithFallback(
                const StraightDistanceMetric &fallback,
                const std::vector<std::vector<DistanceMatrix>>& distanceMatrix);

        InClusterPrecomputedDistanceMetricWithFallback(
                const ClusterHierarchy& clusters,
                const std::vector<std::vector<DistanceMatrix>>& distanceMatrix);

        virtual double distance(const StateOrClusterRef& a,
                              const StateOrClusterRef& b,
                              size_t layer_id,
                              size_t cluster_id) const override;
    };

    /**
     *
     * Iteratively descend the hierarchy of clusters, and determine a good traversal order.
     *
     * @tparam P        The type of "point" to use, such as a robot configuration or a Eigen::Vector2d.
     * @param start_pos The start point of the tour.
     * @param hierarchy A cluster hierarchy to attempt to traverse.
     *
     * @return A vector of indices into the most detailed level of the hierarchy, indicative of visitation order.
     */
    std::vector<size_t> determine_visitation_order(const ompl::base::ScopedStatePtr &start_pos,
                                                   const ClusterHierarchy &hierarchy,
                                                   const ClusterDistanceFn &distanceFn);

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
