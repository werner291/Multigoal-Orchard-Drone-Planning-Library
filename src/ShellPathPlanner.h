
#ifndef NEW_PLANNERS_SHELLPATHPLANNER_H
#define NEW_PLANNERS_SHELLPATHPLANNER_H

#include <range/v3/view/enumerate.hpp>
#include "NewMultiGoalPlanner.h"
#include "SphereShell.h"
#include "DistanceHeuristics.h"

class ShellPathPlanner : public NewMultiGoalPlanner {

    std::shared_ptr<CollisionFreeShell> shell;
    std::shared_ptr<OmplDistanceHeuristics> distance_heuristics;
    bool apply_shellstate_optimization;


public:
    ShellPathPlanner(std::shared_ptr<CollisionFreeShell> shell,
                     bool applyShellstateOptimization,
                     std::shared_ptr<OmplDistanceHeuristics> distanceHeuristics);

    PlanResult plan(const ompl::base::SpaceInformationPtr &si,
                    const ompl::base::State *start,
                    const std::vector<ompl::base::GoalPtr> &goals,
                    SingleGoalPlannerMethods &methods) override;

    PlanResult assembleFullPath(
            const ompl::base::SpaceInformationPtr &si,
            const std::vector<ompl::base::GoalPtr> &goals,
            OMPLSphereShellWrapper &ompl_shell,
            const std::vector<std::pair<size_t, ompl::geometric::PathGeometric>> &approaches,
            const std::vector<size_t> &ordering, PlanResult &result,
            ompl::geometric::PathGeometric &initial_approach) const;

    ompl::geometric::PathGeometric retreat_move_probe(
            const std::vector<ompl::base::GoalPtr> &goals,
            OMPLSphereShellWrapper &ompl_shell,
            PlanResult &result,
            ompl::geometric::PathGeometric &goal_to_goal,
            const std::pair<size_t, ompl::geometric::PathGeometric> &approach_a,
            const std::pair<size_t, ompl::geometric::PathGeometric> &approach_b) const;

    static std::optional<ompl::geometric::PathGeometric> planFirstApproach(
            const ompl::base::State *start,
            SingleGoalPlannerMethods &methods,
            ompl::geometric::PathGeometric &approach_path);

    std::vector<size_t> computeApproachOrdering(
            const ompl::base::State *start,
            const std::vector<ompl::base::GoalPtr> &goals,
            const std::vector<std::pair<size_t, ompl::geometric::PathGeometric>> &approaches) const;

    std::vector<std::pair<size_t, ompl::geometric::PathGeometric>> planApproaches(
            const ompl::base::SpaceInformationPtr &si,
            const std::vector<ompl::base::GoalPtr> &goals,
            SingleGoalPlannerMethods &methods,
            const OMPLSphereShellWrapper &ompl_shell) const;

    std::optional<ompl::geometric::PathGeometric> planApproachForGoal(
            const ompl::base::SpaceInformationPtr &si,
            SingleGoalPlannerMethods &methods,
            const OMPLSphereShellWrapper &ompl_shell,
            const ompl::base::GoalPtr &goal) const;

    Json::Value parameters() const override;

};

#endif //NEW_PLANNERS_SHELLPATHPLANNER_H
