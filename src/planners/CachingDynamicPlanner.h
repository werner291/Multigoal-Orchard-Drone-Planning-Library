// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_CACHINGDYNAMICPLANNER_H
#define NEW_PLANNERS_CACHINGDYNAMICPLANNER_H

#include "../DynamicMultiGoalPlanner.h"

#include "shell_path_planner/ApproachPlanning.h"
#include "shell_path_planner/Construction.h"

#include "../IncrementalTSPMethods.h"

/**
 * Implementation of the DynamicMultiGoalPlanner interface which is essentially a dynamic version of ShellPathPlanner.
 *
 * It works by computing an approach path for every goal that is either given to the plan method or added to the set of
 * goals via the replan* methods.
 *
 * The approach paths are then ordered using a TSP method, and the planner yields a path segment to the first goal in the
 * ordering, or nullopt if the planner has decided to halt.
 *
 * @tparam ShellPoint
 */
template<typename ShellPoint>
class CachingDynamicPlanner : public DynamicMultiGoalPlanner {

	/**
	 * @brief A struct containing a goal and an approach path to that goal.
	 */
	struct ApproachToGoal {
		ompl::base::GoalPtr goal;
		OmplApproachPath<ShellPoint> approach;
	};

	/// The ApproachPlanningMethods object used to compute approach paths.
	std::shared_ptr<ApproachPlanningMethods<ShellPoint>> approach_planner;

	/// The IncrementalTSPMethods object used to reorder the approach paths.
	std::shared_ptr<IncrementalTSPMethods> tsp_method;

	/// The ShellBuilder that is used to construct the shell space from the planning scene.
	/// since it is only invoked on first path computation, we do store it here instead of
	/// just using it in the constructor.
	MkOmplShellFn<ShellPoint> shellBuilder;

	/// The shell space; will be initialized on first path computation, nullptr until then.
	std::shared_ptr<OmplShellSpace<ShellPoint>> shell_space;

	/// The path last used by the robot to move away from the shell and approach some goal;
	/// as such, to get back to the shell, we can usually just reverse this path. (this is checked)
	std::optional<OmplApproachPath<ShellPoint>> to_shell_cache;

	/**
	 * @brief Computes a path from a given state to the shell; will use the to_shell_cache if possible.
	 *
	 * @warning By convention of the OmplApproachPath class, the path returned by this method will start
	 * at the shell and end at the given state, and must thus be reversed to be used as a retreat path.
	 *
	 * @param si 		The space information pointer.
	 * @param start 	The start state from which to compute the path.
	 * @return 			The path to the shell, or nullopt if no path could be found.
	 */
	std::optional<OmplApproachPath<ShellPoint>>
	find_path_to_shell(const ompl::base::SpaceInformationPtr &si, const ompl::base::State *start);

	/**
	 * The vector of ApproachToGoal objects in the currently-determined visitation order.
	 *
	 * Unless new goals are added, this vector will not change, and the planner will simply
	 * visit the goals in the order they are stored in this vector.
	 */
	std::vector<ApproachToGoal> ordering;

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

	/**
	 * A small struct that pairs the last path emitted by the planner with the goal it was emitted for.
	 */
	struct LastEmitted {
		ompl::geometric::PathGeometric path; //< The last path emitted by the planner.
		ompl::base::GoalPtr goal;        //< The goal the last path was emitted for.
	};

	/// The last path emitted by the planner, and the goal it was emitted for, if any. Otherwise nullopt.
	std::optional<LastEmitted> last_emitted_path;

	/**
     * @brief Determine a new ordering with insertion of a given approach path.
     *
     * This method calculates a new ordering of the approach paths based on the TSP method after inserting a new approach path.
     * It uses the distances between the current approach path and the other approach paths to calculate the cost of inserting
     * a new approach path at a particular location in the ordering. The TSP method then determines the new ordering of the approach
     * paths with the lowest cost of insertion.
     *
     * @tparam ShellPoint The type of shell point used in the planner.
     * @param approach The approach path to be inserted.
     * @return A vector of NewOrderingEntry structs representing the new ordering of the approach paths.
     */
	[[nodiscard]] std::vector<IncrementalTSPMethods::NewOrderingEntry>
	determine_new_ordering_with_insertion(const OmplApproachPath<ShellPoint> &approach) const;

	/**
	 * @brief Given an interrupt and a current state, return a path segment starting from that interrupt according to the current approach ordering.
	 *
	 * @param si 					The space information pointer.
	 * @param current_state 		The current state.
	 * @param interrupt 			The interrupt.
	 * @return 						A path segment starting from the interrupt, or nullopt if no path could be found.
	 */
	[[nodiscard]] std::optional<DynamicMultiGoalPlanner::PathSegment> continueFromInterrupt(const ompl::base::SpaceInformationPtr &si,
																			  const ompl::base::State *current_state,
																			  const PathInterrupt &interrupt);

public:
	explicit CachingDynamicPlanner(const std::shared_ptr<ApproachPlanningMethods<ShellPoint>> &approachPlanner,
								   const std::shared_ptr<IncrementalTSPMethods> &tspMethod,
								   MkOmplShellFn<ShellPoint> shellBuilder);

	virtual ~CachingDynamicPlanner() = default;

	/**
	 * @brief Computes a path segment to one of the goals, and records the given set of goals.
	 *
	 * @param si 				The space information pointer.
	 * @param start 			The start state.
	 * @param goals 			The set of goals.
	 * @param planning_scene 	The planning scene.
	 * @return 					A path segment to one of the goals, or nullopt if the planner has decided to halt.
	 */
	[[nodiscard]] std::optional<PathSegment> plan_initial(const ompl::base::SpaceInformationPtr &si,
											const ompl::base::State *start,
											const std::vector<ompl::base::GoalPtr> &goals,
											const AppleTreePlanningScene &planning_scene,
											double padding) override;

	/**
	 * @brief Replans after a successful visit to a goal. Typically, this will just be the next section in the precomputed plan.
	 *
	 * @param si 					The space information pointer.
	 * @param current_state 		The current state of the robot; typically the last state in the path segment returned by the last call to plan or replan.
	 * @param planning_scene 		The planning scene.
	 * @return 						A path segment to one of the goals, or nullopt if the planner has decided to halt.
	 */
	[[nodiscard]] std::optional<PathSegment> replan_after_path_end(const ompl::base::SpaceInformationPtr &si,
													 const ompl::base::State *current_state,
													 const AppleTreePlanningScene &planning_scene) override;

	/**
	 * @brief Replans after discovering a new goal.
	 *
	 * @param si 						The space information pointer.
	 * @param current_state 			The current state of the robot; typically at some arbitrary point on the path segment returned by the last call to plan or replan.
	 * @param new_goal 					The new goal.
	 * @param interrupt 				The time of the interrupt of the previous path segment when the new goal was discovered.
	 * @param planning_scene 			The planning scene.
	 * @return 							A path segment to the new goal, or nullopt if the planner has decided to halt.
	 */
	[[nodiscard]] std::optional<PathSegment> replan_after_discovery(const ompl::base::SpaceInformationPtr &si,
													  const ompl::base::State *current_state,
													  const ompl::base::GoalPtr &new_goal,
													  const PathInterrupt &interrupt,
													  const AppleTreePlanningScene &planning_scene) override;

	/**
	 * @brief Replans after removing a goal.
	 *
	 * @param si 					The space information pointer.
	 * @param current_state 		The current state of the robot; typically at some arbitrary point on the path segment returned by the last call to plan or replan.
	 * @param removed_goal 			The removed goal.
	 * @param interrupt 			The time of the interrupt of the previous path segment when the goal was removed.
	 * @param planning_scene 		The planning scene.
	 * @return 						A path segment to one of the goals, or nullopt if the planner has decided to halt.
	 */
	[[nodiscard]] std::optional<PathSegment> replan_after_removal(const ompl::base::SpaceInformationPtr &si,
													const ompl::base::State *current_state,
													const ompl::base::GoalPtr &removed_goal,
													const PathInterrupt &interrupt,
													const AppleTreePlanningScene &planning_scene) override;


};

#endif //NEW_PLANNERS_CACHINGDYNAMICPLANNER_H
