// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3-5-23.
//

#ifndef NEW_PLANNERS_MIDPOINT_PULL_H

#include <optional>

#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/Goal.h>

#include "../planners/MultiGoalPlanner.h"

namespace mgodpl::midpoint_pull {
	/**
		 * @brief Perform the midpoint pull operation on a triplet of states, smoothing out that section of the path.
		 *
		 * @param[in] state1 First state in the triplet.
		 * @param[in] state2 Second state in the triplet (the state to be "pulled").
		 * @param[in] state3 Third state in the triplet.
		 * @param[out] output The resulting state after applying the midpoint pull operation.
		 * @param[in] si Shared pointer to the SpaceInformation, containing the state space and other relevant information.
		 * @param[in] t Interpolation parameter t, within range [0, 1].
		 *
		 * @pre t is within range [0, 1].
		 * @post output is the result of the midpoint pull operation, an interpolation between state2 and the midpoint of state1 and state3.
		 *
		 * @warning This function does not check for motion validity between the states.
		 */
	void midpointPull(const ompl::base::State *state1,
					  const ompl::base::State *state2,
					  const ompl::base::State *state3,
					  ompl::base::State *output,
					  const ompl::base::SpaceInformationPtr &si,
					  const double t);

	/**
		 * @brief Apply the midpoint pull operation to a path at a given index i, checking for motion validity.
		 *
		 * @param[in,out] path The path to be modified.
		 * @param[in] index The index of the state in the path to be "pulled" (i.e., the state to be modified).
		 * @param[in] t Interpolation parameter t, within range [0, 1]. 0 corresponds to the original state,
		 * 			    1 to the midpoint of the states before and after the state to be modified.
		 *
		 * @return True if the operation was successful and the path was modified, false otherwise.
		 *
		 * @pre index is within range [1, path.getStateCount() - 2].
		 * @pre t is within range [0, 1].
		 * @post The path is modified only if the new path segments are valid, i.e., no collisions are introduced.
		 */
	bool applyMidpointPullToPath(ompl::geometric::PathGeometric &path, int index, double t);

	/**
		 * @brief Apply the midpoint pull operation to the path in an attempt to shorten it while ensuring all goal regions are visited.
		 *
		 * @param[in,out] path The path to be modified.
		 * @param[in] G A vector of pairs, where each pair consists of an index into the path and a pointer to the corresponding goal region.
		 * @param[in] projectGoalRegion A function to project a new state onto the goal region it should visit.
		 *
		 * @return True if the operation was successful and the path was shortened, false otherwise.
		 *
		 * @note The operation is performed for all configurations in the path, except for the first and last configurations.
		 * @note If a configuration is associated with a goal region, the new state is projected onto the goal region after the midpoint pull operation.
		 * @note The configuration is replaced with the new state only if the path is shortened and remains collision-free.
		 */
	bool midpointPullTryShortenPath(ompl::geometric::PathGeometric &path,
									const std::vector<std::pair<int, ompl::base::GoalPtr>> &G,
									std::function<void(ompl::base::State *,
													   const ompl::base::GoalPtr &)> projectGoalRegion,
									double t);

	/**
		 * @brief Extract the index-goal pairing from the given PlanResult.
		 *
		 * @param[in] planResult The result of a planning query.
		 * @param[in] goals A vector of shared pointers to the goal regions.
		 *
		 * @return A vector of pairs, where each pair consists of an index into the combined path and a shared pointer to the corresponding goal region.
		 *
		 * @note Each PathSegment in the PlanResult is assumed to end with the associated goal.
		 * @note The index in each pair corresponds to the final state of a PathSegment in the combined path.
		 */
	std::vector<std::pair<int, ompl::base::GoalPtr>>
	extractGoalIndexPairing(const MultiGoalPlanner::PlanResult &planResult,
							const std::vector<ompl::base::GoalPtr> &goals);


	/**
		 * @brief A struct that holds parameters for the path shortening process.
		 */
	struct PathShorteningParameters {
		double initialT = 1.0;                      ///< Initial t parameter for the midpoint pull operation
		double minimumT = 1e-6;                     ///< Minimum allowed t value; iteration stops if t falls below this
		double improvementThresholdPercentage = 1.0; ///< Improvement threshold as a percentage of the initial path length
	};

	/**
		 * @brief A class that implements a path shortening algorithm using the midpoint pull operation.
		 */
	class PathShorteningAlgorithm {
		ompl::geometric::PathGeometric &path_; ///< The path to be shortened
		const MultiGoalPlanner::PlanResult &plan_result_; ///< The planning result associated with the path
		const std::vector<ompl::base::GoalPtr> &goals_; ///< The goal regions
		std::function<void(ompl::base::State *,
						   const ompl::base::GoalPtr &)> projectGoalRegion_; ///< Function to project a state onto a goal region
		PathShorteningParameters params_; ///< Parameters for the path shortening process
		double t_; ///< Current t value
		double lastLength_; ///< Length of the path from the last iteration
		double improvementThreshold_; ///< Current improvement threshold

	public:
		/**
		 * @brief Constructs a PathShorteningAlgorithm object.
		 *
		 * @param path The PathGeometric object to be shortened.
		 * @param plan_result The planning result associated with the path.
		 * @param goals The vector of goal regions.
		 * @param projectGoalRegion A function that projects a state onto a goal region.
		 * @param params The parameters for the path shortening process.
		 */
		PathShorteningAlgorithm(ompl::geometric::PathGeometric &path,
								const MultiGoalPlanner::PlanResult &plan_result,
								const std::vector<ompl::base::GoalPtr> &goals,
								std::function<void(ompl::base::State *, const ompl::base::GoalPtr &)> projectGoalRegion,
								const PathShorteningParameters &params = PathShorteningParameters());

		/**
		 * @brief Runs the full path shortening algorithm.
		 *
		 * @return The length of the path after running the algorithm.
		 */
		double run();

		/**
		 * @brief Returns the path resulting from the algorithm.
		 *
		 * @return The path resulting from the algorithm.
		 */
		const ompl::geometric::PathGeometric &getResultingPath() const;

	private:
		/**
		 * @brief Runs a single iteration of the path shortening algorithm.
		 *
		 * @return The length of the path after the iteration.
		 */
		double iterate();

		/**
		 * @brief Checks if another iteration can be performed.
		 *
		 * @return True if another iteration can be performed, false otherwise.
		 */
		[[nodiscard]] bool canIterate() const;
	};
}


#define NEW_PLANNERS_MIDPOINT_PULL_H

#endif //NEW_PLANNERS_MIDPOINT_PULL_H
