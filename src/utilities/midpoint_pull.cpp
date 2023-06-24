// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3-5-23.
//

#include "midpoint_pull.h"

namespace MgODPL::midpoint_pull {

	void midpointPull(const ompl::base::State *state1,
					  const ompl::base::State *state2,
					  const ompl::base::State *state3,
					  ompl::base::State *output,
					  const ompl::base::SpaceInformationPtr &si,
					  const double t) {
		ompl::base::ScopedState<> midpoint(si);

		// Compute the midpoint of the two states
		si->getStateSpace()->interpolate(state1, state3, 0.5, midpoint.get());

		// Interpolate between the midpoint and the second state
		si->getStateSpace()->interpolate(state2, midpoint.get(), t, output);
	}

	bool applyMidpointPullToPath(ompl::geometric::PathGeometric &path, int index, double t) {
		if (index < 1 || index >= path.getStateCount() - 1) {
			throw std::invalid_argument("Invalid index for midpoint pull operation");
		}

		ompl::base::SpaceInformationPtr spaceInfo = path.getSpaceInformation();
		ompl::base::State *state1 = path.getState(index - 1);
		ompl::base::State *state2 = path.getState(index);
		ompl::base::State *state3 = path.getState(index + 1);

		ompl::base::ScopedState<> newState(spaceInfo);
		midpointPull(state1, state2, state3, newState.get(), spaceInfo, t);

		// Check for motion validity between the new state and its neighbors
		if (!spaceInfo->checkMotion(state1, newState.get()) || !spaceInfo->checkMotion(newState.get(), state3)) {
			return false;
		}

		spaceInfo->copyState(state2, newState.get());

		return true;
	}

	bool midpointPullTryShortenPath(ompl::geometric::PathGeometric &path,
									const std::vector<std::pair<int, ompl::base::GoalPtr>> &G,
									std::function<void(ompl::base::State *,
													   const ompl::base::GoalPtr &)> projectGoalRegion,
									double t) {

		bool pathShortened = false;

		// Create a vector of optionals for quick lookup of associated goal regions
		std::vector<std::optional<ompl::base::GoalPtr>> goalRegions(path.getStateCount());
		for (const auto &pair: G) {
			goalRegions[pair.first] = pair.second;
		}

		// Iterate over all configurations in the path (except for the first and last configurations)
		for (auto i = 1; i < path.getStateCount() - 1; ++i) {

			auto si = path.getSpaceInformation();
			auto *state1 = path.getState(i - 1);
			auto *state2 = path.getState(i);
			auto *state3 = path.getState(i + 1);

			// Perform the midpoint pull operation
			ompl::base::ScopedState<> newState(si);
			midpointPull(state1, state2, state3, newState.get(), si, t);

			// If the configuration is associated with a goal region, project the new state onto the goal region
			if (goalRegions[i].has_value()) {
				projectGoalRegion(newState.get(), *goalRegions[i]);
			}

			// Calculate the original and new distances
			auto originalDistance = si->distance(state1, state2) + si->distance(state2, state3);
			auto newDistance = si->distance(state1, newState.get()) + si->distance(newState.get(), state3);

			// If the new path is shorter and collision-free, replace the configuration with the new state
			if (newDistance < originalDistance && si->checkMotion(state1, newState.get()) &&
				si->checkMotion(newState.get(), state3)) {
				si->copyState(state2, newState.get());
				pathShortened = true;
			}
		}

		return pathShortened;
	}

	std::vector<std::pair<int, ompl::base::GoalPtr>>
	extractGoalIndexPairing(const MultiGoalPlanner::PlanResult &planResult,
							const std::vector<ompl::base::GoalPtr> &goals) {
		std::vector<std::pair<int, ompl::base::GoalPtr>> goalIndexPairing;

		// Start the index at -1 because we will increment it by the path length at the start of the loop.
		int currentIndex = -1;
		for (const auto &segment: planResult.segments) {
			// Increment the current index by the number of states in the current path segment
			currentIndex += segment.path_.getStateCount();

			// Add a pair consisting of the current index and the associated goal region to the goal-index pairing
			goalIndexPairing.emplace_back(currentIndex, goals[segment.to_goal_id_]);
		}

		return goalIndexPairing;
	}


	PathShorteningAlgorithm::PathShorteningAlgorithm(ompl::geometric::PathGeometric &path,
													 const MultiGoalPlanner::PlanResult &plan_result,
													 const std::vector<ompl::base::GoalPtr> &goals,
													 std::function<void(ompl::base::State *,
																		const ompl::base::GoalPtr &)> projectGoalRegion,
													 const PathShorteningParameters &params)
			: path_(path),
			  plan_result_(plan_result),
			  goals_(goals),
			  projectGoalRegion_(projectGoalRegion),
			  params_(params) {
		t_ = params_.initialT;
		lastLength_ = path_.length();
		improvementThreshold_ = params_.improvementThresholdPercentage / 100 * lastLength_;
	}

	double PathShorteningAlgorithm::run() {
		while (canIterate()) {
			iterate();
		}
		return path_.length();
	}

	const ompl::geometric::PathGeometric &PathShorteningAlgorithm::getResultingPath() const {
		return path_;
	}

	double PathShorteningAlgorithm::iterate() {
		bool improved = midpointPullTryShortenPath(path_,
												   extractGoalIndexPairing(plan_result_, goals_),
												   projectGoalRegion_,
												   t_);
		double currentLength = path_.length();

		double absoluteImprovement = lastLength_ - currentLength;

		std::cout << "Path length after: " << currentLength << std::endl;

		if (!improved || absoluteImprovement < improvementThreshold_) {
			t_ *= 0.5;
		}

		lastLength_ = currentLength;

		return path_.length();
	}

	bool PathShorteningAlgorithm::canIterate() const {
		return t_ >= params_.minimumT;
	}

}