
#ifndef NEW_PLANNERS_VISITATION_ORDER_STRATEGY_H
#define NEW_PLANNERS_VISITATION_ORDER_STRATEGY_H

#include "../general_utilities.h"
#include "clustering_types.h"
#include "in_cluster_distances.h"

namespace clustering {

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
     *
     * Iteratively descend the hierarchy of clusters, and determine a good traversal order.
     *
     * @tparam P        The type of "point" to use, such as a robot configuration or a Eigen::Vector2d.
     * @param start_pos The start point of the tour.
     * @param hierarchy A cluster hierarchy to attempt to traverse.
     *
     * @return A vector of indices into the most detailed level of the hierarchy, indicative of visitation order.
     */
    std::vector<std::vector<size_t>> determine_visitation_order(const ompl::base::ScopedStatePtr &start_pos,
                                                                const ClusterHierarchy &hierarchy,
                                                                const ClusterDistanceFn &distanceFn);


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
            const std::function<void(const clustering::VisitationOrderSolution &sol)> &solution_callback
    ) {

        // Generate an empty solution, that is either zero-cost, or goes from the start point to the end point.
        solution_callback({{}, end_point ? distance({start_point}, {*end_point}) : 0.0, {}});

        generate_combinations<size_t, clustering::VisitationOrderSolution>(
                index_vector(visitable),
                {{}, 0.0},
                [&](auto begin, auto end, const clustering::VisitationOrderSolution &cost) {

                    // Grab a copy (TODO: maybe a way to optimize by avoiding the copy? {Persistent datastructures may help})
                    clustering::VisitationOrderSolution new_cost = cost;

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
}



#endif //NEW_PLANNERS_VISITATION_ORDER_STRATEGY_H
