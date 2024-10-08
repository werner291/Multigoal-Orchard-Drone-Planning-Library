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
#include "../planning/probing_motions.h"
#include "../planning/cgal_chull_shortest_paths.h"

import approach_by_pullout;
import rrt;
import sampling;
import goal_sampling;
import approach_makeshift_prm;
import collision_detection;
import functional_utils;
import shell_state_projection;

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


/**
 * @brief Tries to pull out a robot from a given goal state.
 *
 * This function attempts to move the robot from a given goal state to a state outside of a tree by moving
 * it along the axis of the robotic arm, thereby minimizing the chance of collision.
 *
 * If the motion does not collide, the function returns a RobotPath from the goal state to a state outside the tree,
 * otherwise it returns an empty optional.
 *
 * @param robot_model 		The robot model to be used.
 * @param chull 			The convex hull of the tree.
 * @param collision_fns 	The collision functions to be used for collision checking.
 * @param goal_sample 		The goal state from which the robot is to be pulled out.
 * @return A RobotPath if the motion does not collide, otherwise an empty optional.
 */
std::optional<RobotPath> try_pullout(
		const robot_model::RobotModel &robot_model,
		const cgal::CgalMeshData &chull,
		CollisionFunctions &collision_fns,
		const RobotState &goal_sample) {

	// Generate the motion without collision checking:
	const auto &path = straightout(
			robot_model,
			goal_sample,
			chull.tree,
			chull.mesh_path
	);

	// This should be a two-state path:
	assert(path.path.states.size() == 2);

	// The first state is the outside state, the second is the goal state:
	auto outside_tree = path.path.states[0];

	// Check for collisions and return the path if it's collision-free.
	if (!collision_fns.motion_collides(goal_sample, outside_tree)) {
		return std::make_optional(RobotPath{.states = {goal_sample, outside_tree}});
	} else {
		return std::nullopt;
	}
}

/**
 * @brief Function that performs probing by pullout approach for planning.
 *
 * This function will repeatedly attempt to sample a goal state. If collision-free,
 * it will attempt to use a pullout motion to generate a path out of the tree.
 *
 * At most 1000 samples are taken for each target point.
 *
 * @param problem		 The ApproachPlanningProblem object containing the robot model and the tree model.
 * @param collision_fns  The CollisionFunctions object used for collision checking.
 * @param rng 			 The RandomNumberGenerator object used for generating random numbers.
 * @return 	An ApproachPlanningResults object containing the generated paths and performance annotations.
 */
ApproachPlanningResults probing_by_pullout(
		const ApproachPlanningProblem &problem,
		CollisionFunctions &collision_fns,
		random_numbers::RandomNumberGenerator &rng) {

	const int MAX_SAMPLES = 1000;

	// Initialize the paths and performance annotations:
	std::vector<std::optional<RobotPath>> paths;
	Json::Value performance_annotations;

	// Create a goal sample function that counts its invocations.
	calls_t goal_samples = 0;
	auto sample_goal = wrap_invocation_counting(goal_region_sampler(problem.robot, rng), goal_samples);

	// For each target point...
	for (const auto &target: problem.tree_model.target_points) {

		// Create a sampler function that generates a random goal state.
		std::function sample_goal_state = [&]() {
			return sample_goal(target);
		};

		// Create a pullout operation function that attempts to pull the robot out of the tree from a given goal state.
		std::function try_pullout_sample = [&](const RobotState &goal_sample) -> std::optional<RobotPath> {
			return try_pullout(problem.robot, *problem.tree_model.tree_convex_hull, collision_fns, goal_sample);
		};

		// Try the pullout operation at valid goal samples.
		auto ap = try_at_valid_goal_samples<RobotPath>(
				collision_fns.state_collides,
				sample_goal_state,
				MAX_SAMPLES,
				try_pullout_sample);

		// Add the path to the list of paths. (It's an optional, so that if it fails, it lines up with the target index.)
		paths.push_back(ap);
	}

	return {
			.paths = paths,
			.performance_annotations = performance_annotations
	};
}

/**
 * @brief Function that performs uses a rapidly-exploring random tree (RRT), rooted in valid goal samples,
 * to plan approach paths in and out of the tree.
 *
 * @param max_goal_samples 		The maximum number of goal samples to be taken.
 * @param max_rrt_iterations 	The maximum number of RRT iterations to be performed.
 * @param sampler_margin 	The margin to be added to the calculated radii for the uniform sampler function.
 * @return The benchmark solver function.
 */
ApproachPlanningMethodFn rrt_from_goal_samples(
		const int max_goal_samples = 1000,
		const int max_rrt_iterations = 1000,
		const double sampler_margin = 2.0
) {
	// Return the benchmark solver function; lambda that captures the algorithm parameters.
	return [=](const ApproachPlanningProblem &problem,
			   CollisionFunctions &collision_fns,
			   random_numbers::RandomNumberGenerator &rng) -> ApproachPlanningResults {

		// Allocate result vector and JSON annotations.
		std::vector<std::optional<RobotPath>> paths;
		Json::Value performance_annotations;

		// Create an invocation-counting goal sampler.
		calls_t goal_samples = 0;
		auto sample_goal = wrap_invocation_counting(goal_region_sampler(problem.robot, rng), goal_samples);

		// Create an accept function that checks if a state is outside the tree.
		auto accept = accept_outside_tree(*problem.tree_model.tree_convex_hull);

		// Create a uniform sampler function of the space around the tree.
		auto sample_state = make_uniform_sampler_fn(problem.robot,
													rng,
													problem.tree_model.tree_mesh.leaves_mesh,
													sampler_margin);

		for (const auto &target: problem.tree_model.target_points) {

			// Create a sampler function that generates a random goal state.
			std::function sample_goal_state = [&]() {
				return sample_goal(target);
			};

			// Define an operation function that attempts to plan an approach path using RRT, with the RRT rooted in the goal state.
			auto try_rrt = [&](const RobotState &goal_sample) {
				return rrt_path_to_acceptable(
						goal_sample,
						sample_state,
						collision_fns.state_collides,
						collision_fns.motion_collides,
						equal_weights_distance,
						max_rrt_iterations,
						accept
				);
			};

			// Try the RRT operation at valid goal samples.
			auto result = try_at_valid_goal_samples<RobotPath>(
					collision_fns.state_collides,
					sample_goal_state,
					max_goal_samples,
					try_rrt);

			// Store the result in the paths vector (nullopt if no path was found, so we line up with the target index).
			paths.push_back(result);
		}

		// Record the number of goal samples taken.
		performance_annotations["goal_samples"] = goal_samples;

		// Return the results.
		return {
				.paths = paths,
				.performance_annotations = performance_annotations
		};
	};
}

/**
 * @brief Function that performs uses a rapidly-exploring random tree (RRT), rooted in valid goal samples,
 * to plan approach paths in and out of the tree.
 *
 * This is a variant of `rrt_from_goal_samples` that uses a biased sampler function to generate samples,
 *
 * @param max_goal_samples 		The maximum number of goal samples to be taken.
 * @param max_rrt_iterations 	The maximum number of RRT iterations to be performed.
 * @param sampler_scale 		The scaling factor to use for the sampler. Larger => Broader area sampled from.
 *
 * @return The benchmark solver function.
 */
ApproachPlanningMethodFn rrt_from_goal_samples_with_bias(
		const int max_goal_samples = 1000,
		const int max_rrt_iterations = 1000,
		const double sampler_scale = 1.0
) {
	return [=](const ApproachPlanningProblem &problem,
			   CollisionFunctions &collision_fns,
			   random_numbers::RandomNumberGenerator &rng) -> ApproachPlanningResults {

		// Allocate result vector and JSON annotations.
		std::vector<std::optional<RobotPath>> paths;
		Json::Value performance_annotations;

		// Create a sample-counting goal sampler.
		calls_t goal_samples = 0;
		auto sample_goal = wrap_invocation_counting(goal_region_sampler(problem.robot, rng), goal_samples);

		// Create an accept function that checks if a state is outside the tree.
		auto accept_at = accept_outside_tree(*problem.tree_model.tree_convex_hull);

		// For every target...
		for (const auto &target: problem.tree_model.target_points) {

			// Create a sampler function for this specific target.
			std::function sample_goal_state = [&]() {
				return sample_goal(target);
			};

			// The biased RRT sampler function that grows an RRT tree from the given goal sample.
			std::function plan_rrt_biased = [&](const RobotState &goal_sample) {

				auto ideal_shell_state = project_to_shell_state(goal_sample,
																*problem.tree_model.tree_convex_hull,
																problem.robot);

				// Create a biased sampler that samples near the (ideal_shell_state -> goal_sample) motion.
				std::function biased_sampler = motionBiasedSampleFn(
						ideal_shell_state,
						goal_sample,
						rng,
						sampler_scale
				);

				// Run the RRT algorithm and try to find a path.
				return rrt_path_to_acceptable(
						goal_sample,
						biased_sampler,
						collision_fns.state_collides,
						collision_fns.motion_collides,
						equal_weights_distance,
						max_rrt_iterations,
						accept_at
				);
			};

			// Try the RRT operation at valid goal samples.
			auto path = try_at_valid_goal_samples<RobotPath>(
					collision_fns.state_collides,
					sample_goal_state,
					max_goal_samples,
					plan_rrt_biased
			);

			// Store the result in the paths vector (nullopt if no path was found, so we line up with the target index).
			paths.push_back(path);
		}

		// Record the number of goal samples taken.
		performance_annotations["goal_samples"] = goal_samples;

		return {
				.paths = paths,
				.performance_annotations = performance_annotations
		};
	};
}

/**
 * This is a "quick and dirty" planning method. It runs in O(1) time.
 *
 * This method places a robot state at an idealized location outside the convex hull of the tree,
 * and then simply checks if a translational motion into the tree towards the target point collides.
 *
 * If not, it returns a path from the idealized state to the target point.
 */
ApproachPlanningResults straight_in(const ApproachPlanningProblem &problem,
									CollisionFunctions &collision_fns,
									random_numbers::RandomNumberGenerator &rng) {

	// Initialize the paths and performance annotations:
	std::vector<std::optional<RobotPath>> paths;
	Json::Value performance_annotations;

	// For every target point...
	for (const auto &target: problem.tree_model.target_points) {

		// Create a side-of-triangle-mesh function for the tree convex hull.
		const auto surface_pt = mgodpl::cgal::from_face_location(
				mgodpl::cgal::locate_nearest(target, *problem.tree_model.tree_convex_hull),
				*problem.tree_model.tree_convex_hull);

		// Compute an idealized shell state.
		RobotState ideal_shell_state = fromEndEffectorAndVector(problem.robot,
																surface_pt.surface_point,
																surface_pt.normal);

		// Project it onto the goal:
		RobotState goal_state = project_to_goal(
				problem.robot,
				ideal_shell_state,
				problem.robot.findLinkByName("flying_base"),
				problem.robot.findLinkByName("end_effector"),
				target);

		// Check if the motion collides:
		if (!collision_fns.motion_collides(ideal_shell_state, goal_state)) {
			paths.push_back(RobotPath{.states = {ideal_shell_state, goal_state}});
		} else {
			paths.push_back(std::nullopt); // No path found; keep a nullopt to align with the index.
		}
	}

	// Return the results.
	return {
			.paths = paths,
			.performance_annotations = performance_annotations
	};
}

REGISTER_BENCHMARK(approach_planning_comparison) {

	// Create a robot model.
	robot_model::RobotModel robot_model = mgodpl::experiments::createProceduralRobotModel(
			{
					.total_arm_length = 1.0, // TODO: Might we want to vary this?
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

	// How many times to repeat each method:
	const size_t REPETITIONS = 2;

	// A list of methods associated with a name.
	std::vector<std::pair<std::string, ApproachPlanningMethodFn>> methods{
			{"pullout",          probing_by_pullout},
			{"rrt_1000",         rrt_from_goal_samples(1000, 1000, 2.0)},
			{"rrt_100",          rrt_from_goal_samples(1000, 100, 2.0)},
			{"rrt_bias_1000",    rrt_from_goal_samples_with_bias(1000, 1000, 2.0)},
			{"rrt_bias_100",     rrt_from_goal_samples_with_bias(1000, 100, 2.0)},
			{"rrt_1000_bm",      rrt_from_goal_samples(1000, 1000, 4.0)},
			{"rrt_100_bm",       rrt_from_goal_samples(1000, 100, 4.0)},
			{"rrt_bias_1000_bm", rrt_from_goal_samples_with_bias(1000, 1000, 4.0)},
			{"rrt_bias_100_bm",  rrt_from_goal_samples_with_bias(1000, 100, 4.0)},
			{"straight_in",      straight_in}
	};

	// Record the list of names, split out from the pairs.
	for (size_t i = 0; i < methods.size(); ++i) {
		results["methods"].append(methods[i].first);
	}

	// Create a list of problems: one for each tree model.
	std::vector<ApproachPlanningProblem> problems;
	problems.reserve(tree_models.size());
	for (const auto &tree_model: tree_models) {
		problems.emplace_back(ApproachPlanningProblem{robot_model, tree_model});

		// Record the problem in the results struct so we can do some analysis later.
		Json::Value problem_json;
		problem_json["name"] = tree_model.tree_model_name;
		problem_json["n_targets"] = static_cast<int>(tree_model.target_points.size());
		results["problems"].append(problem_json);
	}

	// A "run" is a combination of a method, a problem, and a repetition.
	struct Run {
		size_t method_index;
		size_t problem_index;
		size_t repetition_index;
	};

	// Take a cartesian product of all methods, problems, and repetitions.
	std::vector<Run> runs;
	for (size_t method_index = 0; method_index < methods.size(); method_index++) {
		for (size_t problem_index = 0; problem_index < problems.size(); problem_index++) {
			for (size_t repetition_index = 0; repetition_index < REPETITIONS; repetition_index++) {
				runs.push_back({method_index, problem_index, repetition_index});
			}
		}
	}

	// Shuffle to prevent biases caused by uneven CPU load during the run. (A fixed seed is used for reproducibility.)
	std::shuffle(runs.begin(), runs.end(), std::mt19937(42)); // NOLINT(*-msc51-cpp)

	// We're going to run experiments in parallel, so we need a mutex to protect the results:
	std::mutex results_mutex;
	std::atomic_int in_flight = 0; // Track how many experiments are running in parallel at any time.

	// Run the experiments in parallel:
	std::for_each(std::execution::par, runs.begin(), runs.end(), [&](const Run &run) {

		// Count up:
		in_flight += 1;

		// Grab the RNG, seed it with the repetition index:
		random_numbers::RandomNumberGenerator rng(run.repetition_index);

		// Print some information about the run:
		std::cout << "Starting run " << run.method_index << " " << run.problem_index << " " << run.repetition_index
				  << std::endl;
		std::cout << "In flight: " << in_flight << std::endl;

		// Look up the method and problem:
		const auto &[_name, method] = methods[run.method_index];
		const auto &problem = problems[run.problem_index];

		// Create the collision functions with counters.
		// We add counters here to give us an alternative measurement of computational cost, to compare against CPU time.
		// The advantage of using this count is that it is deterministic, and independent of the CPU load or slow algorithm implementation.
		//
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

		// Time elapsed in milliseconds: (Longer is more expensive)
		auto time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

		// Record the results in a JSON object:
		Json::Value result_json;
		result_json["method"] = run.method_index;
		result_json["problem"] = run.problem_index;
		result_json["repetition"] = run.repetition_index;
		result_json["motions_checked"] = motion_collision_check_invocations;
		result_json["states_checked"] = collision_check_invocations;

		result_json["time_ms"] = time_ms;

		// Number of targets reached is the number of paths that are not nullopt:
		result_json["targets_reached"] = std::count_if(result.paths.begin(), result.paths.end(),
													   [](const auto &path) { return path.has_value(); });

		// Record any method-specific annotations.
		result_json["annotations"] = result.performance_annotations;

		// Record the lengths and number of waypoints of each path:
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

		// Store the results.
		{
			// Gotta lock the mutex to prevent threads from clobbering the results.
			std::lock_guard lock(results_mutex);
			results["results"].append(result_json);

			// Print our progress.
			std::cout << "Finished run " << run.method_index << " " << run.problem_index << " " << run.repetition_index
					  << ", completed " << results["results"].size() << " of " << runs.size() << std::endl;
		}

		// Count down:
		in_flight -= 1;
	});
}