#include "MultiGoalPlanner.h"

double MultiGoalPlanner::PlanResult::length() const {
    double length = 0;
    for (const auto &segment : segments) {
        length += segment.path_.length();
    }
    return length;
}

ompl::geometric::PathGeometric MultiGoalPlanner::PlanResult::combined() const {

	ompl::geometric::PathGeometric combined_path(segments[0].path_.getSpaceInformation());

	for (const auto &segment : segments) {
		combined_path.append(segment.path_);
	}

	return combined_path;

}
