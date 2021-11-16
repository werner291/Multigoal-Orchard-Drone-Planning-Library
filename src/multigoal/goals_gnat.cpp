
#include "goals_gnat.h"

bool GNATNode::operator==(const GNATNode &other) const {
    return goal == other.goal && goal_pos == other.goal_pos;
}

bool GNATNode::operator!=(const GNATNode &other) const {
    return !(*this == other);
}


