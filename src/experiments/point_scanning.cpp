#include <json/json.h>
#include <fstream>
#include <algorithm>
#include <mutex>
#include <execution>
#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../experiment_utils/scan_paths.h"
#include "../experiment_utils/joint_distances.h"
#include "../experiment_utils/point_scanning_evaluation.h"
#include "../experiment_utils/parameter_space.h"
#include "../experiment_utils/parametric_paths.h"
#include "../planning/probing_motions.h"
#include "../planning/state_tools.h"
#include "../experiment_utils/declarative/SolutionMethod.h"

using namespace mgodpl;
using namespace declarative;
using namespace experiments;

int main() {

	const StaticPointScanMetaParameters meta_params = {
			.n_repeat = 1,
			.seed = 42,
			.leaf_sizes = {/*0.0, 0.5, 1.0,*/ 1.5 /* , 2.0 */}
	};

	// Generate our environmental parameter space to evaluate/test.
	const auto &eval_params = gen_eval_params(meta_params);

	std::cout << "Will run experiment with meta-parameters:" << std::endl;
	std::cout << toJson(meta_params) << std::endl;

	std::cout << "Will test the following " << eval_params.size() << " scenarios:" << std::endl;
	for (const auto &params: eval_params) {
		std::cout << toJson(params) << std::endl;
	}

	// Get a set of orbits to serve as potential paths to evaluate.
	std::vector<SolutionMethod> orbits;

	for (double radius : {1.0, 1.5, 2.0}) {
		orbits.emplace_back(OrbitFacingTree{OrbitPathParameters{CircularOrbitParameters{.radius = radius}}});
	}

	for (double radius : {1.0, 1.5, 2.0}) {
		for (double amplitude : {0.25, 0.5, 0.75}) {
			for (unsigned int cycles : {4, 8}) {
				orbits.emplace_back(OrbitFacingTree{OrbitPathParameters{SphericalOscillationParameters{.radius = radius, .amplitude = amplitude, .cycles = cycles}}});
			}
		}
	}

	std::cout << "with the following " << orbits.size() << " orbits:" << std::endl;
	for (const auto &orbit: orbits) {
		std::cout << toJson(orbit) << std::endl;
	}

	// Initialize a random number generator
	random_numbers::RandomNumberGenerator rng;

	// A JSON object to store the results
	Json::Value stats;

	// Create an environment cache to store the non-POD environments we create
	mgodpl::experiments::TreeModelCache environment_cache {};

	std::cout << "Starting evaluation" << std::endl;
	std::cout << "Will test " << eval_params.size() << " scenarios with " << orbits.size() << " orbits each, for total of " << eval_params.size() * orbits.size() << " evaluations." << std::endl;

	std::mutex results_mutex;

	size_t scenarios_completed = 0;
	std::atomic_int in_flight = 0;
	size_t total_completed = 0;

	// RobotPath final_path = plan_multigoal_path(robot, tree_model, initial_state);

	// Iterate over every combination of environment and solution.
    std::for_each(std::execution::par, eval_params.begin(), eval_params.end(), [&](const auto &scenario_params) {

		std::cout << "Starting scenario " << (scenarios_completed+1) << " of " << eval_params.size() << std::endl;

		Json::Value scenario_stats;
		scenario_stats["parameters"] = toJson(scenario_params);

		size_t orbits_completed = 0;

		std::for_each(std::execution::par, orbits.begin(), orbits.end(), [&](const auto &orbit) {

			std::cout << "Starting orbit " << (orbits_completed+1) << " of " << orbits.size() << " for scenario " << (scenarios_completed+1) << " of " << eval_params.size() << std::endl;
			std::cout << "In flight: " << (++in_flight) << std::endl;

			// Instantiate the environment for this scenario
			const PointScanEnvironment &env = create_environment(scenario_params, environment_cache);

			// Define the speed of interpolation
			double interpolation_speed = 0.02;

			RobotState initial_state = fromEndEffectorAndVector(env.robot, {0, 5, 5}, {0, 1, 1});

			// Generate the path to evaluate.
			RobotPath path;

			if (auto facing_tree = std::get_if<OrbitFacingTree>(&orbit)) {

				path = parametricPathToRobotPath(env.robot,
												  env.tree_model->leaves_aabb.center(),
												  instantiatePath(
														  facing_tree->params, env.tree_model->leaves_aabb.center(),
														  env.tree_model->canopy_radius), 1000);

			} else if (std::get_if<ProbingMotionsMethod>(&orbit)) {
				 path = plan_multigoal_path(env.robot, env.tree_model->meshes, initial_state);
			}

			// Evaluate it.
			const auto& result = eval_static_path(path, interpolation_speed, scenario_params, env);

			// Create a results JSON object for this run
			Json::Value run;

			// Store the parameters alongside the results for easier analysis
			run["orbit"] = toJson(orbit);
			run["result"] = toJson(result);

			// Store this run in the overall results
			{
				std::lock_guard<std::mutex> lock(results_mutex);
				scenario_stats["attempts"].append(run);

				orbits_completed++;
				std::cout << "Completed " << orbits_completed << " of " << orbits.size() << " orbits. (Scenario " << (scenarios_completed+1) << " of " << eval_params.size() << ")" << std::endl;
				std::cout << "In flight: " << (--in_flight) << std::endl;
				std::cout << "Total: " << (++total_completed) << " of " << (eval_params.size() * orbits.size()) << " evaluations completed." << std::endl;
			}
		});

		{
			std::lock_guard<std::mutex> lock(results_mutex);
			stats.append(scenario_stats);

			scenarios_completed++;
			std::cout << "Completed " << scenarios_completed << " of " << eval_params.size() << " scenarios." << std::endl;
		}
	});

	// Write the JSON object to a file
	Json::StreamWriterBuilder writer;
	writer.settings_["indentation"] = ""; // No indentation.
	std::ofstream file("../analysis/data/point_scanning.json");
	file << Json::writeString(writer, stats);

	std::cout << "All runs completed and written to file." << std::endl;
	std::cout << "Results written to ../analysis/data/point_scanning.json" << std::endl;

}
