// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#include <execution>
#include <random>
#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>

#include "benchmark_function_macros.h"
#include "../experiment_utils/tree_benchmark_data.h"
#include "../planning/RobotPath.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../planning/RandomNumberGenerator.h"
#include "../planning/state_tools.h"
#include "../planning/goal_sampling.h"
#include "../planning/collision_detection.h"

import approach_by_pullout;
import rrt;
import goal_sampling;

#include <CGAL/Side_of_triangle_mesh.h>

using namespace mgodpl;

struct ApproachPlanningProblem {
	const robot_model::RobotModel &robot;
	const experiments::TreeModelBenchmarkData &tree_model;
};

struct ApproachPlanningResults {
	std::vector<std::optional<RobotPath>> paths;
	Json::Value performance_annotations;
};

using ApproachPlanningMethodFn = std::function<ApproachPlanningResults(const ApproachPlanningProblem &,
																	   random_numbers::RandomNumberGenerator &)>;

ApproachPlanningMethodFn probing_by_pullout() {
	return [](const ApproachPlanningProblem &problem,
			  random_numbers::RandomNumberGenerator &rng) -> ApproachPlanningResults {

		std::vector<std::optional<RobotPath>> paths;

		for (const auto &target: problem.tree_model.target_points) {
			auto ap = approach_planning::plan_approach_by_pullout(
					*problem.tree_model.tree_collision_object,
					*problem.tree_model.tree_convex_hull,
					target,
					0.1,
					problem.robot,
					rng,
					1000
			);

			if (ap) {
				paths.emplace_back(ap->path);
			} else {
				paths.emplace_back(std::nullopt);
			}
		}

		return {
				.paths = paths
		};
	};
}

ApproachPlanningMethodFn rrt_from_goal_samples() {
	return [](const ApproachPlanningProblem &problem,
			  random_numbers::RandomNumberGenerator &rng) -> ApproachPlanningResults {
		std::vector<std::optional<RobotPath>> paths;
		Json::Value performance_annotations;

		// Get the AABB of the leaves:
		math::AABBd leaves_aabb = mesh_aabb(problem.tree_model.tree_mesh.leaves_mesh);

		const double ROBOT_SIZE = 2.0;

		double h_radius = std::max({leaves_aabb._max.x(), leaves_aabb._max.y(),
								   std::abs(leaves_aabb._min.x()),
								   std::abs(leaves_aabb._min.y())}) + ROBOT_SIZE;

		double v_radius = leaves_aabb._max.z() + ROBOT_SIZE;

		for (const auto &target: problem.tree_model.target_points) {

			const mgodpl::robot_model::RobotModel &robot_model = problem.robot;
			const auto &tree_collision = *problem.tree_model.tree_collision_object;

			CGAL::Side_of_triangle_mesh<cgal::Surface_mesh, cgal::K> inside(
					problem.tree_model.tree_convex_hull->convex_hull);

			std::function sample_state = [&]() {
				return generateUniformRandomState(robot_model, rng, h_radius, v_radius, M_PI_2);
			};

			std::function sample_goal_state = [&]() {
				return genGoalStateUniform(
						rng,
						target,
						0.0,
						robot_model,
						robot_model.findLinkByName("flying_base"),
						robot_model.findLinkByName("end_effector")
				);
			};

			std::function state_collides = [&](const RobotState &from) {
				return check_robot_collision(robot_model, tree_collision, from);
			};

			std::function motion_collides = [&](const RobotState &from, const RobotState &to) {
				return check_motion_collides(robot_model, tree_collision, from, to);
			};

			paths.push_back(
					try_at_valid_goal_samples<RobotPath>(
					state_collides,
					sample_goal_state,
					1000,
					[&](const RobotState &goal_sample) {
						assert(!state_collides(goal_sample));
						return rrt_path_to_acceptable(
								goal_sample,
								sample_state,
								state_collides,
								motion_collides,
								equal_weights_distance,
								1000,
								[&](const RobotState& state) {
									// TODO: this is technically not 100% accurate but if we get to this point the planning problem is trivial.
									return inside(cgal::to_cgal_point(state.base_tf.translation)) == CGAL::ON_UNBOUNDED_SIDE;
								}
						);
					})
			);
		}

		return {
				.paths = paths,
				.performance_annotations = performance_annotations
		};

	};
}

REGISTER_BENCHMARK(approach_planning_comparison) {

	robot_model::RobotModel robot_model = mgodpl::experiments::createProceduralRobotModel(
			{
					.total_arm_length = 1.0,
					.joint_types = {experiments::JointType::HORIZONTAL},
					.add_spherical_wrist = false
			});

	// Grab a list of all tree models:
	auto tree_models = experiments::loadAllTreeBenchmarkData(results);

	// Drop all targets already outside the tree:
	for (auto &tree_model: tree_models) {
		CGAL::Side_of_triangle_mesh<cgal::Surface_mesh, cgal::K> inside_outside_check(
				tree_model.tree_convex_hull->convex_hull);

		erase_if(tree_model.target_points, [&](const math::Vec3d &target) {
			return inside_outside_check(cgal::to_cgal_point(target)) == CGAL::ON_UNBOUNDED_SIDE;
		});
	}

	// If this is a debug build, drop all by the first three tree models:
#ifndef NDEBUG
	tree_models.erase(tree_models.begin() + 3, tree_models.end());
#endif

	const size_t REPETITIONS = 2;

	std::vector<std::pair<std::string, ApproachPlanningMethodFn>> methods{
		{"probing_by_pullout", probing_by_pullout()},
		{"rrt_from_goal_samples", rrt_from_goal_samples()}
	};

	for (size_t i = 0; i < methods.size(); ++i) {
		results["methods"].append(methods[i].first);
	}

	std::vector<ApproachPlanningProblem> problems;
	problems.reserve(tree_models.size());
	for (const auto &tree_model: tree_models) {
		problems.emplace_back(ApproachPlanningProblem{robot_model, tree_model});

		Json::Value problem_json;
		problem_json["name"] = tree_model.tree_model_name;
		problem_json["n_targets"] = static_cast<int>(tree_model.target_points.size());

		results["problems"].append(problem_json);
	}

	struct Run {
		size_t method_index;
		size_t problem_index;
		size_t repetition_index;
	};

	std::vector<Run> runs;
	for (size_t method_index = 0; method_index < methods.size(); method_index++) {
		for (size_t problem_index = 0; problem_index < problems.size(); problem_index++) {
			for (size_t repetition_index = 0; repetition_index < REPETITIONS; repetition_index++) {
				runs.push_back({method_index, problem_index, repetition_index});
			}
		}
	}

	std::shuffle(runs.begin(), runs.end(), std::mt19937(42));

	std::mutex results_mutex;
	std::atomic_int in_flight = 0;

	std::for_each(std::execution::par, runs.begin(), runs.end(), [&](const Run &run) {
		in_flight += 1;

		// Grab the RNG, seed it with the repetition index:
		random_numbers::RandomNumberGenerator rng(run.repetition_index);

		std::cout << "Starting run " << run.method_index << " " << run.problem_index << " " << run.repetition_index
				  << std::endl;
		std::cout << "In flight: " << in_flight << std::endl;

		const auto &[_name, method] = methods[run.method_index];
		const auto &problem = problems[run.problem_index];

		// Record start time:
		auto start_time = std::chrono::high_resolution_clock::now();
		const auto &result = method(problem, rng);
		// Record end time:
		auto end_time = std::chrono::high_resolution_clock::now();

		// Time elapsed in milliseconds:
		auto time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

		Json::Value result_json;
		result_json["method"] = run.method_index;
		result_json["problem"] = run.problem_index;
		result_json["repetition"] = run.repetition_index;

		result_json["time_ms"] = time_ms;
		result_json["targets_reached"] = std::count_if(result.paths.begin(), result.paths.end(),
													   [](const auto &path) { return path.has_value(); });

		result_json["annotations"] = result.performance_annotations;

		{
			std::lock_guard lock(results_mutex);
			results["results"].append(result_json);

			std::cout << "Finished run " << run.method_index << " " << run.problem_index << " " << run.repetition_index
					  << ", completed " << results["results"].size() << " of " << runs.size() << std::endl;
		}

		in_flight -= 1;
	});
}