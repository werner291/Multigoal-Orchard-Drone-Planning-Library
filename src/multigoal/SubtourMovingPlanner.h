#ifndef NEW_PLANNERS_SUBTOURMOVINGPLANNER_H
#define NEW_PLANNERS_SUBTOURMOVINGPLANNER_H

#include "multi_goal_planners.h"

namespace multigoal {

    class SubtourMovingPlanner : public MultiGoalPlanner {

        std::shared_ptr<MultiGoalPlanner> initialAttemptPlanner_;

    public:
        MultiGoalPlanResult plan(GoalSet &goals, const ompl::base::State *start_state,
                                 PointToPointPlanner &point_to_point_planner,
                                 std::chrono::milliseconds time_budget) override;

        std::string getName() override;

    };
}

#endif //NEW_PLANNERS_SUBTOURMOVINGPLANNER_H
