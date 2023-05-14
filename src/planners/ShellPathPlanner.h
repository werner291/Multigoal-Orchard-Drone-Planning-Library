
#ifndef NEW_PLANNERS_SHELLPATHPLANNER_H
#define NEW_PLANNERS_SHELLPATHPLANNER_H

#include "MultiGoalPlanner.h"
#include "../shell_space/OmplShellSpace.h"
#include "../DistanceHeuristics.h"
#include "../planning_scene_diff_message.h"
#include "shell_path_planner/ApproachPlanning.h"

/**
 * A method/strategy for constructing a OmplShellSpace from scene information.
 */
template <typename ShellPoint>
using MkOmplShellFn = std::function<std::shared_ptr<OmplShellSpace<ShellPoint>>(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr&)>;

enum DistancePredictionStrategy {

	SHELL_PATH_LENGTH, UNOPTIMIZED_GTG_PATH_LENGTH, OPTIMIZED_GTG_PATH_LENGTH,

};

/**
 * @class ShellPathPlanner
 * @brief A planner that computes a path to navigate between a set of goals, using a shell path approach.
 *
 * Given a set of goals and an initial configuration, this planner computes an approach path to each goal,
 * and then plans a path between them by following the shell path. The order in which the goals are visited
 * is determined by solving the traveling salesman problem (TSP) and optimizing the path between each successive pair of goals.
 *
 * This planner uses the following steps:
 * 1. For every goal region, compute an approach path.
 * 2. Plan an initial approach path from the initial configuration to any state in the shell space.
 * 3. Solve the TSP to determine the order in which to visit all reachable goals.
 * 4. Compute an optimized path between each successive pair of goals.
 * 5. Concatenate all paths together to form a complete path.
 *
 * This class is templated on the type of the shell points. It implements the MultiGoalPlanner interface and
 * provides a plan() method that generates a path plan given a start state, a set of goals, and a planning scene.
 * The planner also provides utility methods to compute approach paths and build goal-to-goal paths.
 *
 * The planner is configured with a shell builder function, an approach planning method, and a boolean flag
 * indicating whether to optimize the segments between goals.
 */
template<typename ShellPoint>
class ShellPathPlanner : public MultiGoalPlanner {

public:
	MkOmplShellFn<ShellPoint> shell_builder;
	const std::unique_ptr<ApproachPlanningMethods<ShellPoint>> methods;
	const bool optimize_segments;
	DistancePredictionStrategy distance_prediction_strategy;

	/**
	 * @brief Constructs a new ShellPathPlanner object.
	 *
	 * @param shellBuilder A shell builder function that takes a planning scene and returns a shell space.
	 * @param methods An approach planning method that generates approach paths.
	 * @param optimizeSegments A boolean flag indicating whether to optimize the segments between goals.
	 */
	ShellPathPlanner(MkOmplShellFn<ShellPoint> shellBuilder,
					 std::unique_ptr<ApproachPlanningMethods<ShellPoint>> methods,
					 bool optimizeSegments,
					 DistancePredictionStrategy distance_prediction_strategy = DistancePredictionStrategy::SHELL_PATH_LENGTH);

	/**
	 * @brief Generates a path given a start state, a set of goals, and a planning scene.
	 *
	 * The path shall start at the start state and, for every reachable goal,
	 * contain at least one state that satisfies the goal region.
	 *
	 * @param si The space information.
	 * @param start The start state.
	 * @param goals The set of goals.
	 * @param planning_scene The planning scene.
	 * @param ptc The planner termination condition.
	 *
	 * @return A PlanResult object containing the path plan.
	 */
	MultiGoalPlanner::PlanResult plan(const ompl::base::SpaceInformationPtr &si,
									  const ompl::base::State *start,
									  const std::vector<ompl::base::GoalPtr> &goals,
									  const AppleTreePlanningScene &planning_scene,
									  ompl::base::PlannerTerminationCondition &ptc) override;

	/**
	 * @brief Builds a goal-to-goal path given a shell space, a set of approach paths, and a goal index.
	 *
	 * The methods works by concatenating the approach paths and the shell path, and then optimizing the path.
	 *
	 * @param si The space information.
	 * @param shell The shell space.
	 * @param approach_path_1 The approach path to the goal that the robot will be retreating from.
	 * @param approach_path_2 The approach path to the goal that the robot will be approaching.
	 *
	 * @return A PathGeometric object containing the goal-to-goal path.
	 */
	[[nodiscard]] ompl::geometric::PathGeometric buildGoalToGoal(const ompl::base::SpaceInformationPtr &si,
																 const OmplShellSpace<ShellPoint> &shell,
																 const OmplApproachPath<ShellPoint> &approach_path_1,
																 const OmplApproachPath<ShellPoint> &approach_path_2) const;

	/**
	 * @brief Builds an initial approach path from the start state to the first goal.
	 *
	 * @param si The space information.
	 * @param shell The shell space.
	 * @param initial_approach The initial approach path.
	 * @param approach_paths The set of approach paths.
	 *
	 * @return A PathGeometric object containing the initial approach path.
	 */
	[[nodiscard]] ompl::geometric::PathGeometric buildInitialApproach(const ompl::base::SpaceInformationPtr &si,
																	  const OmplShellSpace<ShellPoint> &shell,
																	  const std::optional<OmplApproachPath<ShellPoint>> &initial_approach,
																	  const OmplApproachPath<ShellPoint>& approach_to_first_goal) const;

	/**
	 * @brief Returns the parameters of the planner as a JSON object.
	 *
	 * @return A JSON object containing the parameters of the planner.
	 */
	[[nodiscard]] Json::Value parameters() const override {
		{
			Json::Value result;
			//
			//		result["shell_builder_params"] = shell_builder->parameters();
			//		result["ptp"] = methods->parameters();
			result["optimize_segments"] = optimize_segments;

			return result;
		}
	}

	/**
	 * @brief Returns the name of the planner.
	 *
	 * @return The name of the planner.
	 */
	[[nodiscard]] std::string name() const {
		return "ShellPathPlanner";
	}

	std::vector<size_t> determineVisitationOrder(const ompl::base::SpaceInformationPtr &si,
												 const std::shared_ptr<OmplShellSpace<ShellPoint>> &shell,
												 const std::optional<OmplApproachPath<ShellPoint>> &initial_approach,
												 const std::vector<std::pair<size_t, OmplApproachPath<ShellPoint>>> &approach_paths) const;
};


#endif //NEW_PLANNERS_SHELLPATHPLANNER_H
