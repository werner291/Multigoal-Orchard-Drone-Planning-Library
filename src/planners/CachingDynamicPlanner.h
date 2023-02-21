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

	/**
	 * @brief Reorders the list of approach paths using a batch TSP algorithm.
	 *
	 * This method takes in an initial approach path and uses a batch TSP algorithm to reorder the list of approach paths
	 * stored in the `ordering` member variable. The ordering is determined by predicting the path length between each
	 * pair of approach paths, using the shell space to compute the path length. The resulting ordering is stored back
	 * into the `ordering` member variable.
	 *
	 * @tparam ShellPoint The shell point type.
	 * @param initial_approach The initial approach path.
	 */
	void batch_reorder(const OmplApproachPath<ShellPoint> &initial_approach);

	/**
	 * @brief Computes an optimized point-to-point path using a caching dynamic planner.
	 *
	 * This method takes in a space information pointer, a retreat path, and an approach path, and uses a non-terminating
	 * condition to compute a shell path. The retreat and approach paths are then concatenated with the shell path to form
	 * a complete point-to-point path. Finally, the complete path is optimized and returned.
	 *
	 * @tparam ShellPoint The shell point type.
	 * @param si The space information pointer.
	 * @param retreat_path The retreat path.
	 * @param approach_path The approach path.
	 * @return The optimized point-to-point path.
	 */
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
