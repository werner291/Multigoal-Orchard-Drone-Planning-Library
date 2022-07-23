#include "MultiGoalPlanner.h"

double MultiGoalPlanner::PlanResult::length() const {
    double length = 0;
    for (const auto &segment : segments) {
        length += segment.path_.length();
    }
    return length;
}
