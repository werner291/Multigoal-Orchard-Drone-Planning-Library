//
// Created by werner on 6-3-23.
//

#ifndef NEW_PLANNERS_INITIALORBITPLANNER_H
#define NEW_PLANNERS_INITIALORBITPLANNER_H

#include "../DynamicMultiGoalPlanner.h"
#include "../RobotPath.h"

/**
 * @class InitialOrbitPlanner
 *
 * A DynamicMultiGoalPlanner that initially returns a path that simply orbits the tree,
 * and then hands off planning to an underlying multigoal planner.
 *
 * TODO: We'll want to turn this into some more generic sequence-based planner.
 *
 */
class InitialOrbitPlanner : public DynamicMultiGoalPlanner {

	const std::shared_ptr<DynamicMultiGoalPlanner> after_planner;

	std::optional<ompl::geometric::PathGeometric> initial_orbit_path;

	std::vector<ompl::base::GoalPtr> batch; // Goals collected during the initial orbit.
public:
	explicit InitialOrbitPlanner(const std::shared_ptr<DynamicMultiGoalPlanner> &staticPlanner);

	std::optional<PathSegment> plan_initial(const ompl::base::SpaceInformationPtr &si,
											const ompl::base::State *start,
											const std::vector<ompl::base::GoalPtr> &goals,
											const AppleTreePlanningScene &planning_scene) override;

	std::optional<PathSegment> replan_after_path_end(const ompl::base::SpaceInformationPtr &si,
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


#endif //NEW_PLANNERS_INITIALORBITPLANNER_H
