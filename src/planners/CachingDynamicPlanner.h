// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_CACHINGDYNAMICPLANNER_H
#define NEW_PLANNERS_CACHINGDYNAMICPLANNER_H

#include "../DynamicMultiGoalPlanner.h"

#include "shell_path_planner/ApproachPlanning.h"
#include "shell_path_planner/Construction.h"

class IncrementalTSPMethods {

public:
	virtual std::vector<size_t> initial_ordering(size_t n,
												 std::function<double(size_t, size_t)> distance,
												 std::function<double(size_t)> first_distance) = 0;

	virtual std::vector<size_t> update_ordering(const std::vector<size_t> &current_ordering,
												size_t new_goal,
												std::function<double(size_t, size_t)> distance,
												std::function<double(size_t)> first_distance) = 0;
};

template<typename ShellPoint>
class CachingDynamicPlanner : public DynamicMultiGoalPlanner {

	struct ApproachToGoal {
		ompl::base::GoalPtr goal;
		std::optional<OmplApproachPath<ShellPoint>> approach;
	};

	std::shared_ptr<ApproachPlanningMethods<ShellPoint>> approach_planner;
	std::shared_ptr<IncrementalTSPMethods> tsp_method;
	MkOmplShellFn<ShellPoint> shellBuilder;
	std::shared_ptr<OmplShellSpace<ShellPoint>> shell_space;

	OmplApproachPath<ShellPoint> to_shell;

	std::vector<ApproachToGoal> ordering;

	void batch_reorder(const OmplApproachPath<ShellPoint> &initial_approach);

	ompl::geometric::PathGeometric optimizedPointToPoint(const ompl::base::SpaceInformationPtr &si,
														 const OmplApproachPath<ShellPoint> &retreat_path,
														 const OmplApproachPath<ShellPoint> &approach_path) const;

public:
	explicit CachingDynamicPlanner(const std::shared_ptr<ApproachPlanningMethods<ShellPoint>> &approachPlanner,
								   const std::shared_ptr<IncrementalTSPMethods> &tspMethod,
								   MkOmplShellFn<ShellPoint> shellBuilder);

	std::optional<PathSegment> plan(const ompl::base::SpaceInformationPtr &si,
									const ompl::base::State *start,
									const std::vector<ompl::base::GoalPtr> &goals,
									const AppleTreePlanningScene &planning_scene) override;

	std::optional<PathSegment> replan_after_successful_visit(const ompl::base::SpaceInformationPtr &si,
															 const ompl::base::State *current_state,
															 const ompl::base::GoalPtr &visited_goal,
															 const AppleTreePlanningScene &planning_scene) override;

	std::optional<PathSegment> replan_after_discovery(const ompl::base::SpaceInformationPtr &si,
													  const ompl::base::State *current_state,
													  const ompl::base::GoalPtr &new_goal,
													  const PathInterrupt &interrupt,
													  const AppleTreePlanningScene &planning_scene) override;

	std::optional<PathSegment> replan_after_removal(const ompl::base::SpaceInformationPtr &si,
													const ompl::base::State *current_state,
													const ompl::base::GoalPtr &removed_goal,
													const PathInterrupt &interrupt,
													const AppleTreePlanningScene &planning_scene) override;

};

#endif //NEW_PLANNERS_CACHINGDYNAMICPLANNER_H
