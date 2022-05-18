#include "NewMultiGoalPlanner.h"

double NewMultiGoalPlanner::PlanResult::length() const {
    double length = 0;
    for (const auto &segment : segments) {
        length += segment.path_.length();
    }
    return length;
}
