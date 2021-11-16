

#include "preorder_strategy.h"
#include "../multigoal/goals_gnat.h"
#include <vector>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <fstream>

PreorderStrategy::Solution KClosestNearestNeighbourOrder::generate_proposals(const Eigen::Vector3d &start_position,
                                                                             const std::vector<Eigen::Vector3d> &target_positions,
                                                                             std::function<bool(
                                                                                     std::vector<size_t>)> callback) {

    std::vector<GNATNode> nearestKToStart;
    buildGoalGNAT<Eigen::Vector3d>(target_positions, [](const Eigen::Vector3d &tgt) { return tgt; })
            .nearestK({SIZE_MAX, start_position}, target_positions.size(), nearestKToStart);

    Solution solution;

    for (const GNATNode &first_point: nearestKToStart) {

        assert(first_point.goal != SIZE_MAX); // Make sure we don't get the dummy lookup value for some weird reason.

        std::vector<size_t> points_in_order{first_point.goal};

        auto nn = buildGoalGNAT<Eigen::Vector3d>(target_positions, [](const Eigen::Vector3d &tgt) { return tgt; });

        nn.remove(first_point);

        while (nn.size() > 0) {
            auto next = nn.nearest({
                                           points_in_order.back(),
                                           target_positions[points_in_order.back()]
                                   });
            nn.remove(next);
            points_in_order.push_back(next.goal);
        }

        solution.ordering = points_in_order;

        if (!callback(points_in_order)) break;
    }

    return solution;

}
