#include <json/json.h>
#include <fstream>
#include <random_numbers/random_numbers.h>
#include <algorithm>
#include <mutex>
#include <execution>
#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../experiment_utils/mesh_utils.h"
#include "../experiment_utils/scan_paths.h"
#include "../experiment_utils/joint_distances.h"
#include "../experiment_utils/point_scanning_evaluation.h"
#include "../experiment_utils/declarative_environment.h"
#include "../experiment_utils/parameter_space.h"
#include "../experiment_utils/parametric_paths.h"
#include "../experiment_utils/LoadedTreeModel.h"

using namespace mgodpl;
using namespace declarative;
using namespace experiments;

void run_experiment(const StaticPointScanMetaParameters &meta_params,
					const std::vector<PointScanEvalParameters> &eval_params,
					const std::vector<OrbitPathParameters> &orbits) {
	std::cout << "Will run experiment with meta-parameters:" << std::endl;
	std::cout << toJson(meta_params) << std::endl;

	std::cout << "Will test the following " << eval_params.size() << " scenarios:" << std::endl;
	for (const auto &params: eval_params) {
		std::cout << toJson(params) << std::endl;
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
	TreeModelCache environment_cache {};

	std::cout << "Starting evaluation" << std::endl;
	std::cout << "Will test " << eval_params.size() << " scenarios with " << orbits.size() << " orbits each, for total of " << eval_params.size() * orbits.size() << " evaluations." << std::endl;

	std::mutex results_mutex;

	size_t scenarios_completed = 0;
	std::atomic_int in_flight = 0;

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
			const auto &env = create_environment(scenario_params, environment_cache);

			// Define the speed of interpolation
			double interpolation_speed = 0.01;

			// Generate the path to evaluate.
			const RobotPath &path = parametricPathToRobotPath(env.robot,
															  env.tree_model->leaves_aabb.center(),
															  instantiatePath(
																	  orbit, env.tree_model->leaves_aabb.center(),
																	  env.tree_model->canopy_radius), 1000);

			// Evaluate it.
			const auto& result = eval_static_path(path,
												  interpolation_speed,
												  env.scannable_points,
												  scenario_params.sensor_params,
												  env.mesh_occlusion_model);

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
	std::ofstream file("../analysis/data/point_scanning.json");
	file << Json::writeString(writer, stats);

	std::cout << "All runs completed and written to file." << std::endl;
	std::cout << "Results written to ../analysis/data/point_scanning.json" << std::endl;
}

int main() {

	const StaticPointScanMetaParameters meta_params {
			.n_repeat = 1,
			.leaf_scales = {0.0, 0.5, 1.0, 1.5, 2.0},
			.seed = 42,
	};

	// Generate our environmental parameter space to evaluate/test.
	random_numbers::RandomNumberGenerator rng(meta_params.seed);

	// Define the sensor parameters
	SensorScalarParameters sensor_params {
			.maxViewDistance = INFINITY,
			.minViewDistance = 0.0,
			.fieldOfViewAngle = M_PI / 3.0,
			.maxScanAngle = M_PI / 3.0,
	};

	// Create a set of trees with different leaf scaling factors
	std::vector<TreeModelParameters> tree_params;

	for (const double leaf_scale: meta_params.leaf_scales) {
		tree_params.push_back(TreeModelParameters {
				.name = "appletree",
				.leaf_scale = leaf_scale,
				.fruit_subset = Unchanged {},
				.seed = rng.uniformInteger(0, std::numeric_limits<int>::max())
		});
	}

	for (const auto &tree_param: tree_params) {
		eval_params.push_back({
			  .tree_params = tree_param,
			  .sensor_params = sensor_params
	    });
	}

	// Get a set of orbits to serve as potential paths to evaluate.
	const std::vector<OrbitPathParameters> orbits{
			{.parameters = CircularOrbitParameters{.radius = 1.0}},
			{.parameters = CircularOrbitParameters{.radius = 1.5}},
			{.parameters = CircularOrbitParameters{.radius = 2.0}},
			{.parameters = SphericalOscillationParameters{.radius = 1.0, .amplitude = 0.5, .cycles = 4}},
			{.parameters = SphericalOscillationParameters{.radius = 1.0, .amplitude = 0.5, .cycles = 8}},
			{.parameters = SphericalOscillationParameters{.radius = 1.5, .amplitude = 0.5, .cycles = 4}},
			{.parameters = SphericalOscillationParameters{.radius = 1.5, .amplitude = 0.5, .cycles = 8}}
	};

	run_experiment(meta_params, eval_params, orbits);

}