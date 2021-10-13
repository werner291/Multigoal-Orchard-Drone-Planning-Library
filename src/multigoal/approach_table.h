
#ifndef NEW_PLANNERS_APPROACH_TABLE_H
#define NEW_PLANNERS_APPROACH_TABLE_H

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/geometric/PathGeometric.h>
#include <unordered_set>

namespace multigoal {

    typedef std::vector<std::shared_ptr<ompl::base::GoalSampleableRegion>> SampleableGoals;

    typedef std::vector<std::vector<ompl::base::ScopedStatePtr>> GoalApproachTable;

    struct Visitation {
        size_t target_idx;
        size_t approach_idx;
    };

    GoalApproachTable takeGoalSamples(const ompl::base::SpaceInformationPtr &si, const SampleableGoals &goals, int k);

    std::vector<Visitation> random_initial_order(const GoalApproachTable &goal_samples);

    void keepBest(const ompl::base::OptimizationObjective &opt, GoalApproachTable &table, int keep_k);

    struct GoalApproach {
        size_t goal_id;
        size_t approach_id;
        ompl::geometric::PathGeometric approach_path;
    };

    class ATSolution {
        std::vector<GoalApproach> solution_;
    public:

        [[nodiscard]] std::vector<GoalApproach> &getSegments();

        [[nodiscard]] const std::vector<GoalApproach> &getSegmentsConst() const;

        [[nodiscard]] std::optional<const ompl::base::State *> getLastState() const;

    private:
        std::shared_ptr<GoalApproachTable> approachTable_;
        ompl::base::SpaceInformationPtr si_;

    public:
        void check_valid() const;
    };

    class Replacement {
        size_t position;
        GoalApproach new_segment;
    };

}

#endif //NEW_PLANNERS_APPROACH_TABLE_H
