
#ifndef NEW_PLANNERS_ATRANDOM_H
#define NEW_PLANNERS_ATRANDOM_H

#include "multi_goal_planners.h"
#include "approach_table.h"

class ATRandom : public MultiGoalPlanner {
public:
    MultiGoalPlanResult plan(const std::vector<GoalSamplerPtr> &goals, const ompl::base::State *start_state,
                             PointToPointPlanner &point_to_point_planner) override;

    std::string getName() override;

    static MultiGoalPlanResult toMultiGoalResult(multigoal::ATSolution &solution);
};


#endif //NEW_PLANNERS_ATRANDOM_H
