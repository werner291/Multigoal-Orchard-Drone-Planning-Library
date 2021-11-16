
#ifndef NEW_PLANNERS_PREORDER_STRATEGY_H
#define NEW_PLANNERS_PREORDER_STRATEGY_H

#include <vector>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

class PreorderStrategy {

public:
    struct Solution {
        std::vector<size_t> ordering;

        [[nodiscard]] double length(const Eigen::Vector3d &start, const std::vector<Eigen::Vector3d> &apples) const {
            assert(!apples.empty());
            double length = (apples[ordering[0]] - start).norm();
            for (size_t idx = 0; idx + 1 < ordering.size(); ++idx) {
                length += (apples[ordering[idx]] - apples[ordering[idx + 1]]).norm();
            }
            return length;
        }

    };

    virtual Solution generate_proposals(const Eigen::Vector3d &start_position,
                                        const std::vector<Eigen::Vector3d> &target_positions,
                                        std::function<bool(std::vector<size_t>)>) = 0;

};

class KClosestNearestNeighbourOrder : public PreorderStrategy {

public:
    Solution generate_proposals(const Eigen::Vector3d &start_position,
                                const std::vector<Eigen::Vector3d> &target_positions,
                                std::function<bool(std::vector<size_t>)> callback) override;
};

#endif //NEW_PLANNERS_PREORDER_STRATEGY_H
