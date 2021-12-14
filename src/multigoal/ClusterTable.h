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
    struct Cluster {
        // The index of the representative goal sample.
        ompl::base::ScopedStatePtr representative;
        // A mapping of indices into a vector containing the items to be clustered, and a cost to reach the indicated item from the representative sample.
        // These indices either refer to goal samples directly on the lowest clustering level,
        // or to sub-clusters on higher levels of the hierarchy.
        std::map<size_t, double> members;
        // A sorted set of goal indices that are reachable through a goal sample within this cluster sub-hierarchy.
        std::set<size_t> goals_reachable;
    };

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


    template<typename T, typename V>
    void generate_combinations
            (std::vector<T> visitable,
             const V empty_value,
             const std::function<V(std::vector<size_t>::const_iterator first,
                                   std::vector<size_t>::const_iterator last,
                                   const V &)> &consider_cb,
             size_t elements_fixed = 0) {


        for (size_t swap_with = elements_fixed; swap_with < visitable.size(); ++swap_with) {
            std::swap(visitable[elements_fixed], visitable[swap_with]);
            V value = consider_cb(visitable.begin(), visitable.begin() + elements_fixed + 1, empty_value);
            generate_combinations(
                    visitable, value, consider_cb, elements_fixed + 1
            );
            std::swap(visitable[elements_fixed], visitable[swap_with]);
        }
    }

    struct VisitationOrderSolution {
        std::vector<size_t> visit_order;
        double cost;
        size_t goals_visited;

        bool is_better_than(const VisitationOrderSolution &sln) {
            return goals_visited > sln.goals_visited || (goals_visited == sln.goals_visited && cost < sln.cost);
        }
    };

    template<typename P, typename T>
    void generate_visitations(
            const P &start_point,
            const std::vector<T> &visitable,
            const std::optional<P> &end_point,
            std::function<double(const std::variant<P, T> a, const std::variant<P, T> b)> distance,
            std::function<const std::vector<size_t> &(const T &a)> reachable,
            const std::function<void(const VisitationOrderSolution &sol)> &solution_callback
    ) {

        struct PartialScore {
            std::unordered_set<size_t> goals_visited;
            double cost{};
        };

        solution_callback({{}, end_point ? distance({start_point}, {*end_point}) : 0.0, 0});

        clustering::generate_combinations<size_t, PartialScore>(
                index_vector(visitable),
                {{}, 0.0},
                [&](auto begin, auto end, const PartialScore &cost) {

                    PartialScore new_cost = cost;

                    if (begin + 1 == end) {
                        new_cost.cost += distance({visitable[*begin]}, {start_point});
                    } else {
                        new_cost.cost += distance({visitable[*(end - 2)]}, {visitable[*(end - 1)]});
                    }

                    const std::vector<size_t> reachable_goals = reachable(visitable[*(end - 1)]);
                    new_cost.goals_visited.insert(reachable_goals.begin(), reachable_goals.end());

                    double total_cost = end_point ? (new_cost.cost + distance({visitable[*(end - 1)]}, {*end_point}))
                                                  : new_cost.cost;

                    // TODO: Avoid the inconsistent use of iterator ranges and copying.
                    solution_callback(VisitationOrderSolution{
                            std::vector(begin, end),
                            total_cost,
                            new_cost.goals_visited.size()
                    });

                    return new_cost;
                });

    }

    template<typename T>
    std::vector<std::vector<size_t>> propose_orders(const T &entry_point,
                                                    const std::optional<T> &exit_point,
                                                    const std::vector<T> &visitable,
                                                    const std::function<double(const T &, const T &)> &distance_fn) {

        std::vector<size_t> visit_order = index_vector<T>(visitable);

        std::random_device rand;

        struct Candidate {
            double cost_estimate{};
            std::vector<size_t> ordering;
        };

        auto cmp = [](const Candidate &a, const Candidate &b) {
            return a.cost_estimate < b.cost_estimate; // TODO Might have to reverse this.
        };

        std::priority_queue<Candidate, std::vector<Candidate>, decltype(cmp)> top_10(cmp);

        do {
            double cost_estimate = distance_fn(entry_point, visitable[0]);

            std::set<size_t> goals_visited;

            for (size_t visit_i = 1; visit_i < visit_order.size(); visit_i++) {
                cost_estimate += distance_fn(visitable[visit_order[visit_i - 1]],
                                             visitable[visit_order[visit_i]]);
                // TODO weigh by the number of goals visited?
            }

            if (exit_point) {
                cost_estimate += distance_fn(visitable[visit_order.back()], *exit_point);
            }

            top_10.push({
                                cost_estimate,
                                visit_order
                        });

            if (top_10.size() > 10) top_10.pop();

        } while (std::next_permutation(visit_order.begin(), visit_order.end()));

        std::vector<std::vector<size_t>> candidates;

        while (!top_10.empty()) {
            candidates.push_back(top_10.top().ordering);
            top_10.pop();
        }

        std::reverse(candidates.begin(), candidates.end());

        return candidates;
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
