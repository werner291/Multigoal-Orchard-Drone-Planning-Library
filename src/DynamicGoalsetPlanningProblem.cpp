// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 24-2-23.
//

#include <range/v3/view/iota.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/cartesian_product.hpp>
#include <range/v3/range/conversion.hpp>
#include "DynamicGoalsetPlanningProblem.h"

std::vector<DynamicGoalsetPlanningProblem>
DynamicGoalsetPlanningProblem::genDynamicGoalsetPlanningProblems(const AppleTreePlanningScene &scene,
																 const moveit::core::RobotModelPtr &robot,
																 int reps) {

	// Generate a sequence of IDs.
	auto rep_ids = ranges::views::iota(0, reps);

	const auto n_apples = {10, 50};

	// Generate a range of probabilities, ranging from 0.0 to 1.0.
	const int N_PROBABILITIES = 5;

	auto probs = ranges::views::iota(0, N_PROBABILITIES) |
				 ranges::views::transform([](int i) { return i / (double) (N_PROBABILITIES - 1); });

	auto combinations = ranges::views::cartesian_product(rep_ids, probs, n_apples);

	return combinations | ranges::views::transform([&](const auto &pair) -> DynamicGoalsetPlanningProblem {
		const auto &[rep_id, prob, n] = pair;
		return {randomStateOutsideTree(robot, rep_id),
				generateAppleDiscoverability((int) scene.apples.size(), prob, rep_id, n)};
	}) | ranges::to_vector;

}
