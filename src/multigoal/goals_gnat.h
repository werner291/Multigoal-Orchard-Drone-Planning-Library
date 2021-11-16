
#ifndef NEW_PLANNERS_GOALS_GNAT_H
#define NEW_PLANNERS_GOALS_GNAT_H

#include <cstddef>
#include <Eigen/Core>
#include "multi_goal_planners.h"

/**
 * A small utility struct that stores a projection of a goal, as well as an index that refers back to that goal in a GoalSet.
 */
struct GNATNode {
    size_t goal{};
    Eigen::Vector3d goal_pos;

    bool operator==(const GNATNode &other) const;

    bool operator!=(const GNATNode &other) const;
};

/**
 * \brief Build a Geometric Nearest-neighbour Access Tree by projecting each goal using the given projection function.
 */
template<typename Goal>
ompl::NearestNeighborsGNAT<GNATNode>
buildGoalGNAT(const std::vector<Goal> &goals,
              const std::function<Eigen::Vector3d(const Goal &)> &goalProjection) {

    ompl::NearestNeighborsGNAT<GNATNode> nn;

    nn.setDistanceFunction([](const GNATNode &a, const GNATNode &b) {
        return (a.goal_pos - b.goal_pos).norm();
    });

    for (size_t idx = 0; idx < goals.size(); ++idx) {
        nn.add({idx, goalProjection(goals[idx])});
    }

    return nn;
}

#endif //NEW_PLANNERS_GOALS_GNAT_H
