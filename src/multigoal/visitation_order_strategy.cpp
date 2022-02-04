#include "visitation_order_strategy.h"

std::vector<size_t>
clustering::visit_clusters_naive(const std::vector<std::vector<Cluster>> &cluster_hierarchy,
                                 const std::vector<StateAtGoal> &goal_samples,
                                 const ompl::base::State *start_state,
                                 const ompl::base::SpaceInformation &si) {

    std::vector<size_t> visit_order;

    struct StackFrame {
        size_t level;
        size_t in_level;
    };

    std::set<size_t> goals_visited;

    std::vector<StackFrame> stack;
    for (size_t i = 0; i < cluster_hierarchy.back().size(); ++i) {
        stack.push_back({cluster_hierarchy.size() - 1,
                         i
                        });
    }

    while (!stack.empty()) {

        StackFrame frame = stack.back();
        stack.pop_back();

        const Cluster &cluster = cluster_hierarchy[frame.level][frame.in_level];

        std::vector<size_t> additional_reachable_goals;

        std::set_difference(
                cluster.goals_reachable.begin(), cluster.goals_reachable.end(),
                goals_visited.begin(), goals_visited.end(),
                std::back_inserter(additional_reachable_goals)
        );

        if (!additional_reachable_goals.empty()) {

            if (frame.level == 0) {
                assert(additional_reachable_goals.size() == 1);
                visit_order.push_back(frame.in_level);
                goals_visited.insert(additional_reachable_goals.begin(), additional_reachable_goals.end());
            } else {
                for (const auto &item: cluster.members) {
                    stack.push_back({
                                            frame.level - 1,
                                            item.first
                                    });
                }
            }
        }
    }

    return visit_order;
};

std::vector<std::vector<size_t>> clustering::determine_visitation_order(const ompl::base::ScopedStatePtr &start_pos,
                                                                        const ClusterHierarchy &hierarchy,
                                                                        const ClusterDistanceFn &distanceFn) {

    assert(hierarchy[0].size() == 1);

    std::vector<std::vector<size_t>> per_layer_orders;

    // Base case: hierarchy is guaranteed to have exactly one cluster at the top.
    std::vector<clustering::Visitation> previous_layer_order = {
            {0, hierarchy[0][0].goals_reachable}
    };

    // Step case: go layer-by-layer down the hierarchy.
    for (size_t layer_i = 1; layer_i < hierarchy.size(); ++layer_i) {

        std::cout << "Ordering on layer " << layer_i << std::endl;

        // Build up a visitation order.
        std::vector<clustering::Visitation> layer_order;

        // Get a reference to the current layer of the hierarchy...
        const std::vector<Cluster> &layer = hierarchy[layer_i];
        // And the lower LOD layer that we just came through.
        const std::vector<Cluster> &previous_layer = hierarchy[layer_i - 1];

        // In the previous step, we determined an order in which to visit the clusters.
        // We will now do so, in that order.
        for (size_t visit_i = 0; visit_i < previous_layer_order.size(); ++visit_i) {

            // We're looking for the optimal order in which to traverse the cluster members.
            clustering::VisitationOrderSolution sub_layer_best_solution{
                    {}, INFINITY, {}
            };

            // The ordering is aware of entry-and-exit-points.
            // For the first in the sequence, that's the starting position of the robot.
            auto entry_point = start_pos;
            if (visit_i > 0) {
                // On subsequent sub-clusters, we use the previous cluster's representative as a reference point.
                entry_point = layer[previous_layer_order[visit_i - 1].subcluster_index].representative;
            }

            // Same for the exit point, except that we don't have a specified end point for the global sequence,
            // so it's only relevant if we know we'll be visiting another cluster after this.
            std::optional<ompl::base::ScopedStatePtr> exit_point;
            if (visit_i + 1 < previous_layer_order.size()) {
                exit_point = layer[previous_layer_order[visit_i + 1].subcluster_index].representative;
            }

            // Get the cluster members and put them in a vector.
            auto members_vec = boost::copy_range<std::vector<size_t>>(
                    previous_layer[previous_layer_order[visit_i].subcluster_index].members
                    | boost::adaptors::map_keys);

            // Distance function. Either looks up the representative of a cluster based on index,
            // or uses the given reference point directly.
            auto distance = [&](const StateOrClusterRef &a, const StateOrClusterRef &b) {
                return distanceFn.distance(a, b, layer_i, previous_layer_order[visit_i].subcluster_index);
            };

            // Look up a cluster's reachable goal identifiers.
            // Note the explicit return type: if not, the set is de-referenced, then a reference
            // is returned to that temporary copy, causing a use-after-free bug.
            auto lookup_reachable = [&](const size_t &a) -> const std::set<size_t> & {
                return layer[a].goals_reachable;
            };

            // Go through every possible permutation of the clusters and keep track of the best.
            generate_visitations<ompl::base::ScopedStatePtr, size_t>(
                    entry_point, members_vec, previous_layer_order[visit_i].goals_to_visit, exit_point,
                    distance,
                    lookup_reachable,
                    [&](auto soln) {
                        if (soln.is_better_than(sub_layer_best_solution)) {
                            sub_layer_best_solution = soln;
                        }
                    });

            // Concatenate to the total solution for this abstraction layer.
            for (const auto &visit: sub_layer_best_solution.visit_order) {
                layer_order.push_back({
                                              // Note: generate_visitations produces a vector of indices into the vector given to it,
                                              // so we need to perform a lookup to translate to cluster layer indices.
                                              members_vec[visit.subcluster_index],
                                              visit.goals_to_visit
                                      });
            }
        }

        previous_layer_order = layer_order;

        per_layer_orders.push_back(
                boost::copy_range<std::vector<size_t>>(
                        layer_order | boost::adaptors::transformed([](const auto &elt){
                            return elt.subcluster_index;
                        })));
    }

    return per_layer_orders;
}


bool clustering::VisitationOrderSolution::is_better_than(const VisitationOrderSolution &sln) const {
    return goals_visited.size() > sln.goals_visited.size() ||
           (goals_visited.size() == sln.goals_visited.size() && cost < sln.cost);
}
