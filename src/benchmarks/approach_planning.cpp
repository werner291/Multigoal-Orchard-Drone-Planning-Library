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
#include "../planning/probing_motions.h"

import approach_by_pullout;
import rrt;
import goal_sampling;
import approach_makeshift_prm;
import collision_detection;
import functional_utils;

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
																	   CollisionFunctions &,
																	   random_numbers::RandomNumberGenerator &)>;

ApproachPlanningMethodFn probing_by_pullout() {
	return [](const ApproachPlanningProblem &problem,
			  CollisionFunctions &collision_fns,
			  random_numbers::RandomNumberGenerator &rng) -> ApproachPlanningResults {

		const mgodpl::robot_model::RobotModel &robot_model = problem.robot;
		const auto &tree_collision = *problem.tree_model.tree_collision_object;

		std::vector<std::optional<RobotPath>> paths;
		Json::Value performance_annotations;

		int goal_samples = 0;

		for (const auto &target: problem.tree_model.target_points) {

			std::function sample_goal_state = [&]() {
				goal_samples += 1;
				return genGoalStateUniform(
						rng,
						target,
						0.0,
						robot_model,
						robot_model.findLinkByName("flying_base"),
						robot_model.findLinkByName("end_effector")
				);
			};

			std::function pullout = [&](const RobotState &goal_sample) {
				const auto &path = straightout(
						robot_model,
						goal_sample,
						problem.tree_model.tree_convex_hull->tree,
						problem.tree_model.tree_convex_hull->mesh_path
				);

				assert(path.path.states.size() == 2);

				return path.path.start();
			};

			const int MAX_SAMPLES = 1000;

			std::function try_pullout = [&](const RobotState &goal_sample) -> std::optional<RobotPath> {
				const auto &outside_tree = pullout(goal_sample);
				if (!collision_fns.motion_collides(goal_sample, outside_tree)) {
					return std::make_optional(RobotPath{.states = {goal_sample, outside_tree}});
				} else {
					return std::nullopt;
				}
			};

			auto ap = try_at_valid_goal_samples<RobotPath>(
					collision_fns.state_collides,
					sample_goal_state,
					MAX_SAMPLES,
					try_pullout);

			paths.push_back(ap);
		}

		return {
				.paths = paths,
				.performance_annotations = performance_annotations
		};
	};
}

ApproachPlanningMethodFn rrt_from_goal_samples(
		const int max_goal_samples = 1000,
		const int max_rrt_iterations = 1000,
		const double sampler_margin = 2.0
) {
	return [=](const ApproachPlanningProblem &problem,
			   CollisionFunctions &collision_fns,
			   random_numbers::RandomNumberGenerator &rng) -> ApproachPlanningResults {
		std::vector<std::optional<RobotPath>> paths;
		Json::Value performance_annotations;

		// Get the AABB of the leaves:
		math::AABBd leaves_aabb = mesh_aabb(problem.tree_model.tree_mesh.leaves_mesh);

		double h_radius = std::max({leaves_aabb._max.x(), leaves_aabb._max.y(),
									std::abs(leaves_aabb._min.x()),
									std::abs(leaves_aabb._min.y())}) + sampler_margin;

		double v_radius = leaves_aabb._max.z() + sampler_margin;

		int goal_samples = 0;

		for (const auto &target: problem.tree_model.target_points) {

			const mgodpl::robot_model::RobotModel &robot_model = problem.robot;
			const auto &tree_collision = *problem.tree_model.tree_collision_object;

			CGAL::Side_of_triangle_mesh<cgal::Surface_mesh, cgal::K> inside(
					problem.tree_model.tree_convex_hull->convex_hull);

			std::function sample_state = [&]() {
				return generateUniformRandomState(robot_model, rng, h_radius, v_radius, M_PI_2);
			};

			std::function sample_goal_state = [&]() {
				goal_samples += 1;
				return genGoalStateUniform(
						rng,
						target,
						0.0,
						robot_model,
						robot_model.findLinkByName("flying_base"),
						robot_model.findLinkByName("end_effector")
				);
			};

			paths.push_back(
					try_at_valid_goal_samples<RobotPath>(
							collision_fns.state_collides,
							sample_goal_state,
							max_goal_samples,
							[&](const RobotState &goal_sample) {
								assert(!state_collides(goal_sample));
								return rrt_path_to_acceptable(
										goal_sample,
										sample_state,
										collision_fns.state_collides,
										collision_fns.motion_collides,
										equal_weights_distance,
										max_rrt_iterations,
										[&](const RobotState &state) {
											// TODO: this is technically not 100% accurate but if we get to this point the planning problem is trivial.
											return inside(cgal::to_cgal_point(state.base_tf.translation)) ==
												   CGAL::ON_UNBOUNDED_SIDE;
										}
								);
							})
			);
		}

		performance_annotations["goal_samples"] = goal_samples;

		return {
				.paths = paths,
				.performance_annotations = performance_annotations
		};

	};
}

ApproachPlanningMethodFn rrt_from_goal_samples_with_bias(
		const int max_goal_samples = 1000,
		const int max_rrt_iterations = 1000,
		const double sampler_scale = 1.0
) {
	return [=](const ApproachPlanningProblem &problem,
			   CollisionFunctions &collision_fns,
			   random_numbers::RandomNumberGenerator &rng) -> ApproachPlanningResults {
		std::vector<std::optional<RobotPath>> paths;
		Json::Value performance_annotations;

		// Get the AABB of the leaves:
		math::AABBd leaves_aabb = mesh_aabb(problem.tree_model.tree_mesh.leaves_mesh);

		int goal_samples = 0;

		for (const auto &target: problem.tree_model.target_points) {

			const mgodpl::robot_model::RobotModel &robot_model = problem.robot;
			const auto &tree_collision = *problem.tree_model.tree_collision_object;

			CGAL::Side_of_triangle_mesh<cgal::Surface_mesh, cgal::K> inside(
					problem.tree_model.tree_convex_hull->convex_hull);

			std::function sample_goal_state = [&]() {
				goal_samples += 1;
				return genGoalStateUniform(
						rng,
						target,
						0.0,
						robot_model,
						robot_model.findLinkByName("flying_base"),
						robot_model.findLinkByName("end_effector")
				);
			};

			paths.push_back(
					try_at_valid_goal_samples<RobotPath>(
							collision_fns.state_collides,
							sample_goal_state,
							max_goal_samples,
							[&](const RobotState &goal_sample) {
								assert(!state_collides(goal_sample));

								const auto surface_pt = mgodpl::cgal::from_face_location(
										mgodpl::cgal::locate_nearest(goal_sample.base_tf.translation,
																	 *problem.tree_model.tree_convex_hull),
										*problem.tree_model.tree_convex_hull);

								RobotState ideal_shell_state = fromEndEffectorAndVector(robot_model,
																						surface_pt.surface_point,
																						surface_pt.normal);

								assert(inside(cgal::to_cgal_point(ideal_shell_state.base_tf.translation)) ==
									   CGAL::ON_UNBOUNDED_SIDE);

								std::function biased_sampler = [&]() {
									return makeshift_exponential_sample_along_motion(
											goal_sample,
											ideal_shell_state,
											rng,
											sampler_scale
									);
								};

								return rrt_path_to_acceptable(
										goal_sample,
										biased_sampler,
										collision_fns.state_collides,
										collision_fns.motion_collides,
										equal_weights_distance,
										max_rrt_iterations,
										[&](const RobotState &state) {
											// TODO: this is technically not 100% accurate but if we get to this point the planning problem is trivial.
											return inside(cgal::to_cgal_point(state.base_tf.translation)) ==
												   CGAL::ON_UNBOUNDED_SIDE;
										}
								);
							})
			);
		}

		performance_annotations["goal_samples"] = goal_samples;

		return {
				.paths = paths,
				.performance_annotations = performance_annotations
		};

	};
}


ApproachPlanningMethodFn straight_in() {
	return [](const ApproachPlanningProblem &problem,
			  CollisionFunctions &collision_fns,
			  random_numbers::RandomNumberGenerator &rng) -> ApproachPlanningResults {
		std::vector<std::optional<RobotPath>> paths;
		Json::Value performance_annotations;

		// Get the AABB of the leaves:
		math::AABBd leaves_aabb = mesh_aabb(problem.tree_model.tree_mesh.leaves_mesh);

		const double ROBOT_SIZE = 2.0;

		for (const auto &target: problem.tree_model.target_points) {

			const mgodpl::robot_model::RobotModel &robot_model = problem.robot;
			const auto &tree_collision = *problem.tree_model.tree_collision_object;

			CGAL::Side_of_triangle_mesh<cgal::Surface_mesh, cgal::K> inside(
					problem.tree_model.tree_convex_hull->convex_hull);

			const auto surface_pt = mgodpl::cgal::from_face_location(
					mgodpl::cgal::locate_nearest(target, *problem.tree_model.tree_convex_hull),
					*problem.tree_model.tree_convex_hull);

			RobotState ideal_shell_state = fromEndEffectorAndVector(robot_model,
																	surface_pt.surface_point,
																	surface_pt.normal);

			// Project it onto the goal:
			RobotState goal_state = project_to_goal(
					robot_model,
					ideal_shell_state,
					robot_model.findLinkByName("flying_base"),
					robot_model.findLinkByName("end_effector"),
					target);

			if (!collision_fns.motion_collides(ideal_shell_state, goal_state)) {
				paths.push_back(RobotPath{.states = {ideal_shell_state, goal_state}});
			} else {
				paths.push_back(std::nullopt);
			}
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

	// If this is a debug build, drop all by the first two tree models:
#ifndef NDEBUG
	tree_models.resize(2);
#endif

	const size_t REPETITIONS = 2;

	std::vector<std::pair<std::string, ApproachPlanningMethodFn>> methods{
			{"pullout",          probing_by_pullout()},
			{"rrt_1000",         rrt_from_goal_samples(1000, 1000, 2.0)},
			{"rrt_100",          rrt_from_goal_samples(1000, 100, 2.0)},
			{"rrt_bias_1000",    rrt_from_goal_samples_with_bias(1000, 1000, 2.0)},
			{"rrt_bias_100",     rrt_from_goal_samples_with_bias(1000, 100, 2.0)},
			{"rrt_1000_bm",      rrt_from_goal_samples(1000, 1000, 4.0)},
			{"rrt_100_bm",       rrt_from_goal_samples(1000, 100, 4.0)},
			{"rrt_bias_1000_bm", rrt_from_goal_samples_with_bias(1000, 1000, 4.0)},
			{"rrt_bias_100_bm",  rrt_from_goal_samples_with_bias(1000, 100, 4.0)},
			{"straight_in",      straight_in()}
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

		// Create the collision functions with counters.
		// Note to future me: don't put this outside the loop, as we need to count invocations per run.
		calls_t collision_check_invocations = 0;
		calls_t motion_collision_check_invocations = 0;
		CollisionFunctions collision_fns =
				collision_functions_in_environment_counting(
						collision_functions_in_environment({problem.robot, *problem.tree_model.tree_collision_object}),
						{collision_check_invocations, motion_collision_check_invocations}
				);

		// Record start time:
		auto start_time = std::chrono::high_resolution_clock::now();
		const auto &result = method(problem, collision_fns, rng);
		// Record end time:
		auto end_time = std::chrono::high_resolution_clock::now();

		// Time elapsed in milliseconds:
		auto time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

		Json::Value result_json;
		result_json["method"] = run.method_index;
		result_json["problem"] = run.problem_index;
		result_json["repetition"] = run.repetition_index;
		result_json["motions_checked"] = motion_collision_check_invocations;
		result_json["states_checked"] = collision_check_invocations;

		result_json["time_ms"] = time_ms;
		result_json["targets_reached"] = std::count_if(result.paths.begin(), result.paths.end(),
													   [](const auto &path) { return path.has_value(); });

		result_json["annotations"] = result.performance_annotations;

		for (const auto &path: result.paths) {
			if (path.has_value()) {
				const auto &p = path.value();
				result_json["path_lengths"].append(p.states.size());
				result_json["path_distances"].append(pathLength(p));
			} else {
				result_json["path_lengths"].append(Json::Value::null);
				result_json["path_distances"].append(Json::Value::null);
			}
		}

		{
			std::lock_guard lock(results_mutex);
			results["results"].append(result_json);

			std::cout << "Finished run " << run.method_index << " " << run.problem_index << " " << run.repetition_index
					  << ", completed " << results["results"].size() << " of " << runs.size() << std::endl;
		}

		in_flight -= 1;
	});
}