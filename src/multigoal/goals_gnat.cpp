
#include "goals_gnat.h"

bool GNATNode::operator==(const GNATNode &other) const {
    return goal == other.goal && goal_pos == other.goal_pos;
}

bool GNATNode::operator!=(const GNATNode &other) const {
    return !(*this == other);
}

ompl::NearestNeighborsGNAT<GNATNode>
buildGoalGNAT(const GoalSet &goals, const std::function<Eigen::Vector3d(const ompl::base::Goal *)> &goalProjection) {

    ompl::NearestNeighborsGNAT<GNATNode> nn;

    nn.setDistanceFunction([](const GNATNode &a, const GNATNode &b) {
        return (a.goal_pos - b.goal_pos).norm();
    });

    for (size_t idx = 0; idx < goals.size(); ++idx) {
        nn.add({idx, goalProjection(goals[idx].get())});
    }

    return nn;
}
