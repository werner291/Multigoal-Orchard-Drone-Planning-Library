
#ifndef NEW_MULTI_GOAL_PLANNER_H
#define NEW_MULTI_GOAL_PLANNER_H

#include <cstddef>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/Goal.h>
#include "SingleGoalPlannerMethods.h"

class NewMultiGoalPlanner {

public:
    struct PathSegment {
        size_t to_goal_id_;
        ompl::geometric::PathGeometric path_;
    };

    struct PlanResult {
        std::vector<PathSegment> segments;

        double length() const;
    };

    virtual PlanResult plan(const ompl::base::SpaceInformationPtr& si,
                            const ompl::base::State* start,
                            const std::vector<ompl::base::GoalPtr> &goals,
                            SingleGoalPlannerMethods& methods) = 0;
};

#endif // NEW_MULTI_GOAL_PLANNER_H