
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

        [[nodiscard]] const std::vector<GoalApproach> &getSegmentsConst() const {
            return solution_;
        }

        [[nodiscard]] std::optional<const ompl::base::State *> getLastState() const {
            if (solution_.empty()) {
                return {};
            } else {
                return {solution_.back().approach_path.getState(
                        solution_.back().approach_path.getStateCount() - 1)};
            }
        }

    private:
        std::shared_ptr<GoalApproachTable> approachTable_;
        ompl::base::SpaceInformationPtr si_;

    public:
        void check_valid() const {

            const double EPSILON = 1e-5;

            std::unordered_set<size_t> targets_visited;

            for (size_t i = 0; i < solution_.size(); i++) {

                auto approach = solution_[i];

                targets_visited.insert(approach.goal_id);

                const ompl::base::State *last_in_path = approach.approach_path.getState(
                        approach.approach_path.getStateCount() - 1);

                // Make sure the IDs point to a valid table entry.
                assert(approachTable_->size() > approach.goal_id);
                assert((*approachTable_)[approach.goal_id].size() > approach.approach_id);

                // Last state in the path must match the entry in the approach table.
                assert(si_->distance(last_in_path, (*approachTable_)[approach.goal_id][approach.approach_id]->get()) <
                       EPSILON);

                if (i + 1 < solution_.size()) {
                    // If there is another segment after this, the start-and-end-states must match up.
                    assert(si_->distance(last_in_path,
                                         approach.approach_path.getState(
                                                 solution_[i + 1].approach_path.getStateCount() - 1)) < EPSILON);
                }

                for (size_t motion_idx = 0; motion_idx + 1 < approach.approach_path.getStateCount(); ++motion_idx) {
                    // Every state-to-state motion in the approach path must be valid.
                    assert(si_->checkMotion(approach.approach_path.getState(motion_idx),
                                            approach.approach_path.getState(motion_idx + 1)));
                }

            }

            // Every target must be visited exactly once.
            // It is possible that the end-effector passes by a goal twice,
            // but this should not be represented at this level of abstraction.
            assert(targets_visited.size() == solution_.size());
        }
    };

}

#endif //NEW_PLANNERS_APPROACH_TABLE_H
