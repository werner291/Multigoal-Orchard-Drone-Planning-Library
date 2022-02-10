//
// Created by werner on 10-02-22.
//

#ifndef NEW_PLANNERS_AGGLOMERATIVE_CLUSTERING_H
#define NEW_PLANNERS_AGGLOMERATIVE_CLUSTERING_H

#include <variant>
#include <ompl/base/ScopedState.h>
#include "multigoal/PointToPointPlanner.h"

namespace agglomerative_clustering {

    // Worth considering: https://www.researchgate.net/publication/220895309_Speeding-Up_Hierarchical_Agglomerative_Clustering_in_Presence_of_Expensive_Metrics

    struct TreeNode {
        size_t dedup;
        ompl::base::ScopedStatePtr representative;
        std::optional<std::pair<std::shared_ptr<TreeNode>,std::shared_ptr<TreeNode>>> children;
    };

    TreeNode enhancedHierarchicalAgglomerativeClustering(const std::vector<ompl::base::ScopedStatePtr>& to_cluster,
                                                         PointToPointPlanner ptp) {

        struct QueueEntry {
            double lower_bound;
            bool is_exact;
            ompl::base::ScopedStatePtr potential_representative;
            std::pair<std::shared_ptr<TreeNode>,std::shared_ptr<TreeNode>> pair;
        };

        std::priority_queue<QueueEntry> queue;

        std::unordered_set<size_t> eliminated;

        while (!queue.empty()) {

            QueueEntry top = queue.top();
            queue.pop();

            // Lazy-delete.
            if (eliminated.count(top.pair.first->dedup) == 1 || eliminated.count(top.pair.second->dedup) == 1) {
                continue;
            }

            if (top.is_exact) {



            } else {
                auto result = ptp.planToOmplState(0.2,top.pair.first->representative->get(),top.pair.second->representative->get());

                if (result) {

                    top.lower_bound = result->length();
                    top.is_exact = true;
                    top.potential_representative = std::make_shared<ompl::base::ScopedState<ompl::base::StateSpace>>(ptp.getPlanner()->getSpaceInformation());
                    ptp.getPlanner()->getSpaceInformation()->copyState(top.potential_representative->get(), result->getState(result->getStateCount()/2));

                } else {
                    ROS_WARN("Dropped a state pair.");
                }

            }

        }

    }

}

#endif //NEW_PLANNERS_AGGLOMERATIVE_CLUSTERING_H
