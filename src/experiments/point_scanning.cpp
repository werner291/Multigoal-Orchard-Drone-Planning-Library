#include <json/json.h>
#include <fstream>
#include <random_numbers/random_numbers.h>
#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../experiment_utils/mesh_utils.h"
#include "../experiment_utils/scan_paths.h"
#include "../experiment_utils/scan_path_generators.h"
#include "../planning/RobotPath.h"
#include "../planning/state_tools.h"
#include "../experiment_utils/leaf_scaling.h"
#include "../experiment_utils/SensorParameters.h"

using namespace mgodpl;

/**
 * A struct of the distances between two robot states on a per-joint basis.
 */
struct JointDistances {
	/// The distance between the base translations
	double translation_distance;
	/// The distance between the base rotations (in ra
	double rotation_distance;
	/// The distance between the angular positions of each joint (absolute value)
	std::vector<double> joint_distances;
};

/**
 * Calculate the distances between two robot states.
 * @param state1 	The first robot state.
 * @param state2 	The second robot state.
 * @return A struct containing the distances between the two states.
 */
[[nodiscard]] JointDistances calculateJointDistances(const RobotState &state1, const RobotState &state2) {
	JointDistances distances;

	distances.translation_distance = (state2.base_tf.translation - state1.base_tf.translation).norm();
	distances.rotation_distance = angular_distance(state2.base_tf.orientation, state1.base_tf.orientation);

	for (size_t i = 0; i < state1.joint_values.size(); ++i) {
		distances.joint_distances.push_back(std::abs(state2.joint_values[i] - state1.joint_values[i]));
	}

	return distances;
}

[[nodiscard]] Json::Value toJson(const JointDistances &distances) {
	Json::Value json;
	json["translation_distance"] = distances.translation_distance;
	json["rotation_distance"] = distances.rotation_distance;
	json["joint_distances"] = Json::arrayValue;
	for (const auto &joint_distance: distances.joint_distances) {
		json["joint_distances"].append(joint_distance);
	}
	return json;
}

/**
 * Creates a seen/unseen status for each scannable point, initialized to false.
 * @param all_scannable_points 		The scannable points for each fruit.
 * @return 							A vector of vectors booleans, same structure as all_scannable_points, initialized to false.
 */
std::vector<std::vector<bool>> init_seen_status(const std::vector<std::vector<SurfacePoint>> &all_scannable_points) {
	std::vector<std::vector<bool>> ever_seen;
	ever_seen.reserve(all_scannable_points.size());
	for (const auto &scannable_points: all_scannable_points) {
		ever_seen.emplace_back(scannable_points.size(), false);
	}
	return ever_seen;
}

/**
 * Set the seen/unseen status for each scannable point to true if it is visible from the current end effector position.
 *
 * Points that are already seen are not updated nor re-evaluated.
 *
 * @param sensor_params 				The scalar parameters for the sensor model.
 * @param mesh_occlusion_model 			The datastructure to use to check for clear sightlines.
 * @param eye_position 					The position of the sensor/eye.
 * @param eye_forward 					The forward vector of the sensor/eye.
 * @param all_scannable_points 			The scannable points for each fruit.
 * @param ever_seen 					The seen/unseen status for each scannable point.
 */
void update_seen(const SensorScalarParameters &sensor_params,
				 const std::shared_ptr<const MeshOcclusionModel> &mesh_occlusion_model,
				 const math::Vec3d &eye_position,
				 const math::Vec3d &eye_forward,
				 const std::vector<std::vector<SurfacePoint>> &all_scannable_points,
				 std::vector<std::vector<bool>> &ever_seen) {
	for (size_t fruit_i = 0; fruit_i < all_scannable_points.size(); ++fruit_i) {
		for (size_t i = 0; i < all_scannable_points[fruit_i].size(); ++i) {
			if (!ever_seen[fruit_i][i] &&
				is_visible(all_scannable_points[fruit_i][i],
						   eye_position,
						   eye_forward,
						   sensor_params.maxViewDistance,
						   sensor_params.minViewDistance,
						   sensor_params.maxScanAngle,
						   sensor_params.fieldOfViewAngle,
						   *mesh_occlusion_model)) {
				ever_seen[fruit_i][i] = true;
			}
		}
	}
}

/**
 * Evaluate a (static) path by simulating a robot moving along it and scanning the environment.
 *
 * At regular intervals, the robot is stopped and the number of scannable points that have been seen is recorded.
 *
 * @param path 							The path to evaluate.
 * @param interpolation_speed 			The step size to use when moving along the path and evaluating the seen points.
 * @param all_scannable_points 			The scannable points for each fruit.
 * @param sensor_params 				The parameters for the sensor.
 * @param mesh_occlusion_model 			The occlusion model for the mesh.
 * @return 								A JSON object containing the results of the evaluation.
 */
Json::Value eval_path(const RobotPath &path,
					  double interpolation_speed,
					  const std::vector<std::vector<SurfacePoint>> &all_scannable_points,
					  const SensorScalarParameters &sensor_params,
					  const std::shared_ptr<const MeshOcclusionModel> &mesh_occlusion_model) {

	// Define the current position on the path
	PathPoint path_point = {0, 0.0};

	std::vector<std::vector<bool>> ever_seen = init_seen_status(all_scannable_points);

	// Initialize an empty JSON object to store the statistics
	Json::Value stats;

	// Put the robot at the current point (the start of the path)
	RobotState last_state = interpolate(path_point, path);

	// Loop until the path is completed; function will return true when the path is completed.
	while (!advancePathPointClamp(path, path_point, interpolation_speed, equal_weights_max_distance)) {

		// Create a JSON object to store the statistics for this frame
		Json::Value frame_stats;

		// Interpolate the robot's state
		auto interpolated_state = interpolate(path_point, path);

		// Record inter-joint distances
		frame_stats["distance_from_last"] = toJson(calculateJointDistances(last_state, interpolated_state));

		// Get the position/forward of the robot's end effector
		const auto &end_effector_position = interpolated_state.base_tf.translation;
		math::Vec3d end_effector_forward = interpolated_state.base_tf.orientation.rotate(math::Vec3d(0, 1, 0));

		// Update the visibility of the scannable points
		update_seen(
				sensor_params,
				mesh_occlusion_model,
				end_effector_position,
				end_effector_forward, all_scannable_points,
				ever_seen);

		// Count the number of points seen for each fruit so far.
		std::vector<size_t> seen_counts;
		for (size_t fruit_i = 0; fruit_i < all_scannable_points.size(); ++fruit_i) {
			seen_counts.push_back(std::count(ever_seen[fruit_i].begin(), ever_seen[fruit_i].end(), true));
		}

		// Add the metrics to the JSON object
		frame_stats["pts_seen"] = Json::arrayValue;
		for (size_t seen_count: seen_counts) {
			frame_stats["pts_seen"].append(seen_count);
		}

		// Add the frame stats to the JSON object
		stats.append(frame_stats);
	}

	// After we're done, return the full trace.
	return stats;
}

/**
 * Convert a ParametricPath to a RobotPath, by sliding the end-effector of the robot along the path,
 * making it always face the center of the tree.
 *
 * @param robot 			The robot model to use.
 * @param tree_center 		The center of the tree.
 * @param euclidean_path 	The parametric path in Euclidean space.
 * @param n_steps 			The number of steps to use to discretize the path.
 * @return 					A RobotPath that follows the given path.
 */
RobotPath parametricPathToRobotPath(const robot_model::RobotModel &robot,
									const math::Vec3d &tree_center,
									const ParametricPath &euclidean_path,
									size_t n_steps) {

	// Create a RobotPath from these states
	RobotPath path;

	// Generate a sequence of time values between 0 and 1
	for (size_t i = 0; i <= n_steps; ++i) {

		// Calculate the time parameter in range [0, 1] (inclusive)
		double t = static_cast<double>(i) / static_cast<double>(n_steps);

		// Call the ParametricPath function to get the position at time t
		math::Vec3d position = euclidean_path(t);

		// Calculate the relative vector from the tree center
		math::Vec3d relative_vector = position - tree_center;

		// Add the state to the path
		path.states.push_back(fromEndEffectorAndVector(robot, position, relative_vector));
	}

	// Return the path
	return path;
}

/**
 * Instantiate a ParametricPath from an OrbitPathParameters.
 *
 * @param orbit 			The orbit parameters to use.
 * @param tree_center 		The center of the tree that these paths are centered around.
 * @param canopy_radius 	The radius of the tree's canopy.
 * @return 					A ParametricPath that represents the given orbit with the given parameters.
 */
ParametricPath instantiatePath(const OrbitPathParameters &orbit,
							   const math::Vec3d &tree_center,
							   const double canopy_radius) {

	// Go by the type of orbit
	if (auto circularOrbitParameters = std::get_if<CircularOrbitParameters>(&orbit.parameters)) {

		assert(circularOrbitParameters->inclination == 0.0); // We don't support inclined orbits yet.
		assert(circularOrbitParameters->ascendingNodeLongitude == 0.0); // We don't support inclined orbits yet.

		// Create a circular orbit
		return fixed_radius_equatorial_orbit(tree_center, circularOrbitParameters->radius * canopy_radius);

	} else if (auto sphericalOscillationParameters = std::get_if<SphericalOscillationParameters>(&orbit.parameters)) {

		// Create a spherical oscillation orbit
		return latitude_oscillation_path(tree_center,
										 sphericalOscillationParameters->radius * canopy_radius,
										 sphericalOscillationParameters->amplitude,
										 sphericalOscillationParameters->cycles);

	}

	// Should not reach here unless we forgot to implement a new orbit type
	throw std::runtime_error("Unimplemented orbit type");
}

/**
 * The result of a tree model loaded from disk, including the geometry and a number of pre-computed properties.
 */
struct LoadedTreeModel {

	/// The tree meshes, including the leaves, fruits, and branches. (TODO: need to have fruit positions be part of this?)
	mgodpl::tree_meshes::TreeMeshes meshes;

	/// The root points of the leaves, for easy re-scaling.
	std::vector<math::Vec3d> root_points;

	/// The axis-aligned bounding box of the leaves.
	math::AABBd leaves_aabb;

	/// A rough measure of the radius of the tree's canopy.
	double canopy_radius;

	/**
	 * Load a tree model from disk by name, and pre-compute some properties.
	 *
	 * See `mgodpl::tree_meshes::loadTreeMeshes` for more information about the naming and loading.
	 *
	 * @param name 	The name of the tree model to load.
	 * @return 		A LoadedTreeModel object representing the tree model.
	 */
	static LoadedTreeModel from_name(const std::string &name) {
		const auto meshes = mgodpl::tree_meshes::loadTreeMeshes(name);
		const auto root_points = leaf_root_points(meshes);
		const auto leaves_aabb = mesh_aabb(meshes.leaves_mesh);
		const auto canopy_radius = std::min(leaves_aabb.size().x(), leaves_aabb.size().y()) / 2.0;

		return {
				.meshes = meshes,
				.root_points = root_points,
				.leaves_aabb = leaves_aabb,
				.canopy_radius = canopy_radius
		};
	}

};

/**
 * The environmental parameters for a given point scanning experiment, assuming a static environment.
 *
 * This serves as a POD specification for the environment and objective of the experiment,
 * and us meant to be stored alongside the results in order to reproduce them and understand
 * the impact of environmental parameters on the results.
 */
struct PointScanEvalParameters {

	/// Parameters about the tree model, including which model to use and the number of fruits to scan.
	TreeModelParameters tree_params;

	/// Parameters about the sensor, including the field of view and the maximum view distance.
	SensorScalarParameters sensor_params;

	/// How many points to use per fruit. (TODO: this is more an implementation detail, does it belong here?)
	size_t n_scannable_points_per_fruit = 200;

};

Json::Value toJson(const PointScanEvalParameters &params) {
	Json::Value json;
	json["tree_params"] = toJson(params.tree_params);
	json["sensor_params"] = toJson(params.sensor_params);
	json["n_scannable_points_per_fruit"] = params.n_scannable_points_per_fruit;
	return json;
}

/**
 * An instantiation of PointScanEvalParameters.
 *
 * Note: all struct members are references to allow re-use of the same objects.
 *
 * This should be read-only and therefore safe to use in a multi-threaded context.
 */
struct PointScanEnvironment {
	/// The robot model to use.
	const robot_model::RobotModel &robot;
	/// The tree model to use, including the meshes and pre-computed properties.
	const std::shared_ptr<const LoadedTreeModel> tree_model;
	/// The mesh of the leaves, possibly re-scaled to simulate different canopy densities.
	const shape_msgs::msg::Mesh scaled_leaves;
	/// The scannable points per fruit, including surface normal. One vector per fruit mesh, corresponding to tree_model.
	const std::vector<std::vector<SurfacePoint>> scannable_points;
	/// The occlusion model to use to accelerate occlusion checks.
	const std::shared_ptr<const MeshOcclusionModel> mesh_occlusion_model;
};

/**
 * This struct serves as a cache to create simulation environments for a given set of parameters.
 */
struct EnvironmentInstanceCache {

	/// A mutex to protect the cache from concurrent access.
	std::mutex mutex;

	/// The robot model to use.
	const robot_model::RobotModel &robot;

	/// A cache map of tree model names to their loaded instances.
	std::unordered_map<std::string, std::shared_ptr<LoadedTreeModel>> tree_models = {};

	/**
	 * Obtain a tree model and precompute some properties; this operation is cached.
	 *
	 * Access to the cache is thread-safe via a mutex.
	 *
	 * @param tree_name		The name of the tree model to load.
	 * @return 				A shared pointer to the loaded tree model.
	 */
	std::shared_ptr<const LoadedTreeModel> obtain_tree_model(const std::string &tree_name) {

		// Lock the mutex for accessing the cache.
		std::lock_guard<std::mutex> lock(mutex);

		// Add a new tree model to the cache if it does not exist.
		if (tree_models.find(tree_name) == tree_models.end()) {
			tree_models.insert({tree_name, std::make_shared<LoadedTreeModel>(LoadedTreeModel::from_name(tree_name))});
		}

		// Return the tree model from the cache.
		return tree_models.find(tree_name)->second;
	}

	/**
	 * Obtain an environment instance for a given set of parameters.
	 *
	 * Parts of it will be retrieved from the cache if it already exists, or created and stored in the cache if it does not.
	 * This is potentially a relatively expensive operation, as it may involve loading meshes from disk and pre-computing properties.
	 * A mutex protects the cache from concurrent access, so this function is thread-safe.
	 *
	 * Caching applies to only the deterministic parts of this operation, not to point sampling.
	 *
	 * @param params 		The parameters to use to create the environment.
	 * @param rng 			A random number generator to use for sampling points.
	 * @return 				A PointScanEnvironment instance for the given parameters.
	 */
	PointScanEnvironment create_environment(const PointScanEvalParameters &params,
											random_numbers::RandomNumberGenerator &rng) {

		// Load the tree model
		std::shared_ptr<const LoadedTreeModel> tree_model = obtain_tree_model(params.tree_params.name);

		// Create the scannable points
		std::vector<std::vector<SurfacePoint>> all_scannable_points;
		for (const auto &fruit_mesh: tree_model->meshes.fruit_meshes) {
			all_scannable_points.push_back(sample_points_on_mesh(rng, fruit_mesh, params.n_scannable_points_per_fruit));
		}

		// Scale the leaves
		const auto scaled_leaves = scale_leaves(tree_model->meshes,
												tree_model->root_points,
												params.tree_params.leaf_scale);

		// Return the environment instance
		return {
				.robot = robot,
				.tree_model = tree_model,
				.scaled_leaves = scaled_leaves,
				.scannable_points = all_scannable_points,
				.mesh_occlusion_model = std::make_shared<MeshOcclusionModel>(scaled_leaves, 0.0)
		};
	}
};

/**
 * Generate a set of evaluation parameters to use for the point scanning experiment.
 *
 * TODO: maybe we can add some meta-parameters to control which set of parameters to generate?
 */
std::vector<PointScanEvalParameters> gen_eval_params() {

	// Define the sensor parameters
	SensorScalarParameters sensor_params {
			.maxViewDistance = INFINITY,
			.minViewDistance = 0.0,
			.fieldOfViewAngle = M_PI / 3.0,
			.maxScanAngle = M_PI / 3.0,
	};

	// Create a set of trees with different leaf scaling factors
	std::vector<TreeModelParameters> tree_params;

	for (const double leaf_scale: {0.0, 0.5, 1.0, 1.5, 2.0}) {

		TreeModelParameters params{
				.name = "appletree",
				.leaf_scale = leaf_scale,
				.fruit_subset = Unchanged{}
		};

		tree_params.push_back(params);

	}

	// Take the cartesian product of the tree and sensor parameters
	std::vector<PointScanEvalParameters> eval_params;

	for (const auto &tree_param: tree_params) {
		eval_params.push_back({
									  .tree_params = tree_param,
									  .sensor_params = sensor_params
							  });
	}

	// Return the generated parameters
	return eval_params;
}

int main() {

	// Generate our environmental parameter space to evaluate/test.
	const auto &eval_params = gen_eval_params();

	// Get a set of orbits to serve as potential paths to evaluate.
	const std::vector<OrbitPathParameters> orbits{
			{.parameters = CircularOrbitParameters{.radius = 1.0}},
			{.parameters = SphericalOscillationParameters{.radius = 1.0, .amplitude = 0.5, .cycles = 4}}
	};

	// Initialize a random number generator
	random_numbers::RandomNumberGenerator rng;

	// Create a robot model
	const auto &robot = experiments::createProceduralRobotModel();

	// A JSON object to store the results
	Json::Value stats;

	// Create an environment cache to store the non-POD environments we create
	EnvironmentInstanceCache environment_cache = {
			.robot = robot,
	};

	// Iterate over every combination of environment and solution. (TODO: parallelize these loops.)
	for (const auto &scenario_params: eval_params) {
		for (const auto &orbit: orbits) {

			// Instantiate the environment for this scenario
			const auto &env = environment_cache.create_environment(scenario_params, rng);

			// Define the speed of interpolation
			double interpolation_speed = 0.01;

			// Generate the path to evaluate.
			const RobotPath &path = parametricPathToRobotPath(robot,
															  env.tree_model->leaves_aabb.center(),
															  instantiatePath(
																	  orbit, env.tree_model->leaves_aabb.center(),
																	  env.tree_model->canopy_radius), 1000);

			// Evaluate it.
			Json::Value result = eval_path(path, interpolation_speed, env.scannable_points, scenario_params.sensor_params, env.mesh_occlusion_model);

			// Create a results JSON object for this run
			Json::Value run;

			// Store the parameters alongside the results for easier analysis
			run["parameters"] = toJson(scenario_params);
			run["orbit"] = toJson(orbit);
			run["result"] = result;

			// Store this run in the overall results
			stats.append(run);
		}
	}

	// Write the JSON object to a file
	Json::StreamWriterBuilder writer;
	std::ofstream file("../analysis/data/point_scanning.json");
	file << Json::writeString(writer, stats);

}