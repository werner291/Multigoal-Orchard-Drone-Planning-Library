
#ifndef NEW_PLANNERS_SHELLPATHPLANNER_H
#define NEW_PLANNERS_SHELLPATHPLANNER_H

#include <range/v3/view/enumerate.hpp>
#include "NewMultiGoalPlanner.h"
#include "SphereShell.h"
#include "DistanceHeuristics.h"
#include "planning_scene_diff_message.h"

class ShellPathPlanner : public NewMultiGoalPlanner {

    std::shared_ptr<SingleGoalPlannerMethods> methods;
    bool apply_shellstate_optimization;

public:
    ShellPathPlanner(bool applyShellstateOptimization,
                     std::shared_ptr<SingleGoalPlannerMethods> methods);

    PlanResult plan(const ompl::base::SpaceInformationPtr &si, const ompl::base::State *start,
                    const std::vector<ompl::base::GoalPtr> &goals,
                    const AppleTreePlanningScene &planning_scene,
					ompl::base::PlannerTerminationCondition& ptc) override;

    PlanResult assembleFullPath(
            const ompl::base::SpaceInformationPtr &si,
            const std::vector<ompl::base::GoalPtr> &goals,
            OMPLSphereShellWrapper &ompl_shell,
            const std::vector<std::pair<size_t, ompl::geometric::PathGeometric>> &approaches,
            const std::vector<size_t> &ordering,
            PlanResult &result,
            ompl::geometric::PathGeometric &initial_approach) const;

    ompl::geometric::PathGeometric retreat_move_probe(
            const std::vector<ompl::base::GoalPtr> &goals,
            OMPLSphereShellWrapper &ompl_shell,
            PlanResult &result,
            ompl::geometric::PathGeometric &goal_to_goal,
            const std::pair<size_t, ompl::geometric::PathGeometric> &approach_a,
            const std::pair<size_t, ompl::geometric::PathGeometric> &approach_b) const;

    std::optional<ompl::geometric::PathGeometric> planFirstApproach(
            const ompl::base::State *start,
            ompl::geometric::PathGeometric &approach_path);

    std::vector<size_t> computeApproachOrdering(
            const ompl::base::State *start,
            const std::vector<ompl::base::GoalPtr> &goals,
            const std::vector<std::pair<size_t, ompl::geometric::PathGeometric>> &approaches,
            const OmplDistanceHeuristics& distance_heuristics) const;

    std::vector<std::pair<size_t, ompl::geometric::PathGeometric>>
	planApproaches(const ompl::base::SpaceInformationPtr &si,
				   const std::vector<ompl::base::GoalPtr> &goals,
				   const OMPLSphereShellWrapper &ompl_shell,
				   ompl::base::PlannerTerminationCondition &ptc) const;

    std::optional<ompl::geometric::PathGeometric> planApproachForGoal(
            const ompl::base::SpaceInformationPtr &si,
            const OMPLSphereShellWrapper &ompl_shell,
            const ompl::base::GoalPtr &goal) const;

    Json::Value parameters() const override;

    std::string name() const override;

};

#endif //NEW_PLANNERS_SHELLPATHPLANNER_H
