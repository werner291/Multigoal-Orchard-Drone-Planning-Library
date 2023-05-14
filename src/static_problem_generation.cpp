// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 14-5-23.
//

#include <range/v3/view/iota.hpp>
#include <range/v3/view/cartesian_product.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include "static_problem_generation.h"
#include "utilities/experiment_utils.h"

std::vector<std::pair<Json::Value, Problem>>
generateStaticPlanningProblems(moveit::core::RobotModelConstPtr robot, int numRepetitions, const std::vector<std::string> &modelNames) {

	// Get the required scenes for each model name.
	const auto scenes = scenes_for_trees(modelNames);

	// Generate a range of repetition indices.
	auto repIds = ranges::views::iota(0, numRepetitions);

	// Generate a vector of problems by calculating the cartesian product of repIds, applesCounts, and scenes.
	std::vector<std::pair<Json::Value, Problem>> problems =
			ranges::views::cartesian_product(repIds, scenes) |
			ranges::views::transform([&](const auto &pair) {

				const auto &[repId, scene] = pair;

				// Get a random sample of apples.
				AppleTreePlanningScene reduced_scene{.scene_msg = scene.scene_msg, .apples = scene.apples};

				// Shuffle the apples to create a random sample.
				std::shuffle(reduced_scene.apples.begin(), reduced_scene.apples.end(), std::mt19937(repId));

				// Get a random start state.
				auto start = randomStateOutsideTree(robot, repId);

				// Create a JSON value to represent the problem.
				Json::Value problem;
				problem["repId"] = repId;
				problem["scene"] = scene.scene_msg->name;

				// Return the pair containing the JSON value and the Problem object.
				return std::make_pair(problem, Problem{.start = std::move(start), .scene = std::move(reduced_scene)});
			}) | ranges::to_vector;

	return problems;
}
