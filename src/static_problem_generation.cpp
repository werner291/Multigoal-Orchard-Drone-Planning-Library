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
				problem["n_total"] = (int) scene.apples.size();

				// Return the pair containing the JSON value and the Problem object.
				return std::make_pair(problem, Problem{.start = std::move(start), .scene = std::move(reduced_scene)});
			}) | ranges::to_vector;

	return problems;
}

std::vector<std::pair<Json::Value, Problem>>
generateStaticOrchardPlanningProblems(const moveit::core::RobotModelPtr &robot,
									  int num_reps,
									  const std::vector<std::string> &model_names,
									  int n_per_scene) {

	const auto scenes = scenes_for_trees(model_names);

	// Generate a range of repetition indices.
	auto repIds = ranges::views::iota(0, num_reps);

	// Preallocate a vector of problems.
	std::vector<std::pair<Json::Value, Problem>> problems;

	// For each repetition...
	for (int repId : repIds) {

		// Pick 5 scenes at random.

		std::vector<AppleTreePlanningScene> reduced_scenes;

		std::sample(scenes.begin(), scenes.end(), std::back_inserter(reduced_scenes), n_per_scene, std::mt19937(repId));

		// Create a combined planning scene:

		AppleTreePlanningScene combined_scene;
		combined_scene.scene_msg = std::make_shared<moveit_msgs::msg::PlanningScene>();

		double offset = 0.0;

		for (const auto &scene : reduced_scenes) {
			combined_scene.scene_msg->name += scene.scene_msg->name + "_";

			for (auto collision_object : scene.scene_msg->world.collision_objects) {
				collision_object.pose.position.x += offset;
				combined_scene.scene_msg->world.collision_objects.push_back(collision_object);
			}

			for (auto apple : scene.apples) {
				apple.center.x() += offset;
				combined_scene.apples.push_back(apple);
			}

			offset += 2.0;
		}

		// Get a random start state.
		// Careful: the trees are elongated here, so we need something custom.

		moveit::core::RobotState start_state(robot);
		start_state.setToRandomPositions(); // This is a starting point, but we need to ensure the robot is upright and outside the orchard row.

		// For simplicity, we'll put the robot at the head of the row; why would it start in the middle?

		// Get a rng
		std::mt19937 rng(repId);
		// And a 0-1 distribution
		std::uniform_real_distribution<double> dist(0.0, 1.0);

		double theta = dist(rng) * M_PI;
		double height = 0.5 + dist(rng) * 1.0;

		Eigen::Vector3d base_translation(-3.0, 0.0, 2.0);
		Eigen::Quaterniond base_rotation(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));

		Eigen::Isometry3d base_pose;
		base_pose.setIdentity();
		base_pose.translate(base_translation);
		base_pose.rotate(base_rotation);

		start_state.setJointPositions("world_joint", base_pose);
		start_state.update();

		std::cout << "Base translation: " << start_state.getGlobalLinkTransform("base_link").translation() << std::endl;
		std::cout << "Up vector: (" << start_state.getGlobalLinkTransform("base_link").rotation() * Eigen::Vector3d::UnitZ() << ")" << std::endl;

		// I'm kinda tempted to start writing the state into the planning scene as well since there's apparently a slot for it.

		Problem problem {
				.start = start_state,
				.scene = combined_scene,
		};

		Json::Value problem_json;
		problem_json["scene"] = combined_scene.scene_msg->name;
		problem_json["n_apples"] = combined_scene.apples.size();

		problems.emplace_back(problem_json, problem);

	}

	return problems;

}
