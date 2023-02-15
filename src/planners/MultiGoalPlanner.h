
#ifndef NEW_MULTI_GOAL_PLANNER_H
#define NEW_MULTI_GOAL_PLANNER_H

#include <cstddef>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/Goal.h>
#include <jsoncpp/json/value.h>
#include "../SingleGoalPlannerMethods.h"
#include "../planning_scene_diff_message.h"

/**
 * A base class for multi-goal planners that can find paths to a set of goals from a given start state.
 */
class MultiGoalPlanner {

public:
	/**
	 * A structure that represents a segment of a path from a start state to a goal.
	 */
	struct PathSegment {
		size_t to_goal_id_; ///< The ID of the goal that the path segment leads to
		ompl::geometric::PathGeometric path_; ///< The path segment itself
	};

	/**
	 * A structure that represents the result of a planning query, which consists of a set of path segments from
	 * the start state to each of the goals.
	 */
	struct PlanResult {
		std::vector<PathSegment> segments; ///< The set of path segments from the start state to each of the goals

		/**
		 * Returns the total length of all the path segments in the plan result.
		 *
		 * @return The total length of all the path segments in the plan result
		 */
		[[nodiscard]] double length() const;

		/**
		 * Returns a single path that combines all the path segments in the plan result, with the last state of
		 * each segment connecting to the first state of the next segment.
		 *
		 * @return A single path that combines all the path segments in the plan result
		 */
		[[nodiscard]] ompl::geometric::PathGeometric combined() const;
	};

	/**
	 * Plans a path to a set of goals from a given start state.
	 *
	 * @param si The space information for the planning problem
	 * @param start The start state
	 * @param goals The set of goals to plan paths to
	 * @param planning_scene The planning scene for collision checking and other environment information
	 * @param ptc The planner termination condition
	 * @return The plan result, consisting of a set of path segments from the start state to each of the goals
	 */
	[[nodiscard]] virtual PlanResult plan(const ompl::base::SpaceInformationPtr &si,
										  const ompl::base::State *start,
										  const std::vector<ompl::base::GoalPtr> &goals,
										  const AppleTreePlanningScene &planning_scene,
										  ompl::base::PlannerTerminationCondition &ptc) = 0;

	/**
	 * Returns the planner parameters as a JSON object.
	 *
	 * @return The planner parameters as a JSON object
	 */
	[[nodiscard]] virtual Json::Value parameters() const = 0;

	/**
	 * Returns the name of the planner.
	 *
	 * @return The name of the planner
	 */
	[[nodiscard]] virtual std::string name() const = 0;
};

#endif // NEW_MULTI_GOAL_PLANNER_H