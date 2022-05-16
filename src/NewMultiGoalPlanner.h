

#include <cstddef>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/Goal.h>

class NewMultiGoalPlanner {

public:
    struct PathSegment {
        size_t to_goal_id_;
        ompl::geometric::PathGeometric path_;
    };

    struct PlanResult {
        std::vector<PathSegment> segments_;
    };

    typedef const std::function<std::optional<ompl::geometric::PathGeometric>(const ompl::base::State *, const ompl::base::GoalPtr&)> StateToGoalFn;
    typedef const std::function<std::optional<ompl::geometric::PathGeometric>(const ompl::base::State *, const ompl::base::State *)> StateToStateFn;

    virtual PlanResult plan(const ompl::base::SpaceInformationPtr& si,
                            const ompl::base::State* start,
                            const std::vector<ompl::base::GoalPtr> &goals,
                            StateToGoalFn plan_state_to_goal,
                            StateToStateFn plan_state_to_state) = 0;
};