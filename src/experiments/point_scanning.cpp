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

Json::Value eval_path(const RobotPath &path,
					  double interpolation_speed,
					  const std::vector<std::vector<SurfacePoint>> &all_scannable_points,
					  const SensorScalarParameters &sensor_params,
					  const std::shared_ptr<const MeshOcclusionModel> &mesh_occlusion_model) {

	// Create a JSON object

	// Define the current position on the path
	PathPoint path_point = {0, 0.0};

	std::vector<std::vector<bool>> ever_seen;
	ever_seen.reserve(all_scannable_points.size());
	for (const auto &scannable_points: all_scannable_points) {
		ever_seen.emplace_back(scannable_points.size(), false);
	}

	Json::Value stats;

	RobotState last_state = interpolate(path_point, path);

	// Loop until the path is completed
	while (!advancePathPointClamp(path, path_point, interpolation_speed, equal_weights_max_distance)) {

		std::cout << "Progress: " << path_point.segment_i << " / " << path.states.size() << std::endl;

		Json::Value frame_stats;

		// Interpolate the robot's state
		auto interpolated_state = interpolate(path_point, path);

		// Record inter-joint distances
		frame_stats["tdist"] = (interpolated_state.base_tf.translation - last_state.base_tf.translation).norm();
		frame_stats["rdist"] = (angular_distance(interpolated_state.base_tf.orientation,
												 last_state.base_tf.orientation));
		frame_stats["jd"] = Json::arrayValue;
		for (size_t i = 0; i < interpolated_state.joint_values.size(); ++i) {
			frame_stats["jd"].append(std::abs(interpolated_state.joint_values[i] - last_state.joint_values[i]));
		}

		// Get the position of the robot's end effector
		const auto &end_effector_position = interpolated_state.base_tf.translation;
		math::Vec3d end_effector_forward = interpolated_state.base_tf.orientation.rotate(math::Vec3d(0, 1, 0));

//		bool is_visible(const SurfacePoint &point,
//						const math::Vec3d &eye_pos,
//						const math::Vec3d &eye_forward,
//						double max_distance,
//						double min_distance,
//						double max_scan_angle,
//						double fov_angle,
//						const std::shared_ptr<MeshOcclusionModel> &mesh_occlusion_model

		// Update the visibility of the scannable points
		for (size_t fruit_i = 0; fruit_i < all_scannable_points.size(); ++fruit_i) {
			for (size_t i = 0; i < all_scannable_points[fruit_i].size(); ++i) {
				if (!ever_seen[fruit_i][i] &&
					is_visible(all_scannable_points[fruit_i][i],
							   end_effector_position,
							   end_effector_forward,
							   sensor_params.maxViewDistance,
							   sensor_params.minViewDistance,
							   sensor_params.maxScanAngle,
							   sensor_params.fieldOfViewAngle,
							   *mesh_occlusion_model)) {
					ever_seen[fruit_i][i] = true;
				}
			}
		}

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

	return stats;
}

RobotPath parametricPathToRobotPath(const robot_model::RobotModel &robot,
									const math::Vec3d &tree_center,
									const ParametricPath &first_orbit) {// Create a RobotPath from these states
	RobotPath path;

	// Define the number of steps to discretize the path
	size_t num_steps = 1000;

	// Generate a sequence of time values between 0 and 1
	for (size_t i = 0; i <= num_steps; ++i) {
		double t = static_cast<double>(i) / static_cast<double>(num_steps);

		// Call the ParametricPath function to get the position at time t
		math::Vec3d position = first_orbit(t);

		// Calculate the relative vector from the tree center
		math::Vec3d relative_vector = position - tree_center;

		// Add the state to the path
		path.states.push_back(fromEndEffectorAndVector(robot, position, relative_vector));
	}
	return path;
}

ParametricPath
instantiatePath(const math::Vec3d &tree_center, const OrbitPathParameters &orbit, const double canopy_radius) {
	ParametricPath euclidean_path;

	if (auto circularOrbitParameters = std::get_if<CircularOrbitParameters>(&orbit.parameters)) {
		euclidean_path = fixed_radius_equatorial_orbit(tree_center, circularOrbitParameters->radius * canopy_radius);
	} else if (auto sphericalOscillationParameters = std::get_if<SphericalOscillationParameters>(&orbit.parameters)) {
		euclidean_path = latitude_oscillation_path(tree_center, sphericalOscillationParameters->radius * canopy_radius,
												   sphericalOscillationParameters->amplitude,
												   sphericalOscillationParameters->cycles);
	}
	return euclidean_path;
}

struct LoadedTreeModel {
	mgodpl::tree_meshes::TreeMeshes meshes;
	std::vector<math::Vec3d> root_points;
	math::AABBd leaves_aabb;
	double canopy_radius;

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

struct PointScanEvalParameters {
	TreeModelParameters tree_params;
	SensorScalarParameters sensor_params;
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
	const robot_model::RobotModel &robot;
	const std::shared_ptr<const LoadedTreeModel> tree_model;
	const shape_msgs::msg::Mesh scaled_leaves;
	const std::vector<std::vector<SurfacePoint>> scannable_points;
	const std::shared_ptr<const MeshOcclusionModel> mesh_occlusion_model;
};

/**
 * This struct serves as a cache to create simulation environments for a given set of parameters.
 */
struct EnvironmentInstanceCache {

	std::mutex mutex;

	const robot_model::RobotModel &robot;
	std::unordered_map<std::string, std::shared_ptr<LoadedTreeModel>> tree_models = {};

	PointScanEnvironment create_environment(const PointScanEvalParameters &params,
												  random_numbers::RandomNumberGenerator &rng) {

		std::shared_ptr<const LoadedTreeModel> tree_model;

		{
			std::lock_guard<std::mutex> lock(mutex);
			if (tree_models.find(params.tree_params.name) == tree_models.end()) {
				auto tree_model = std::make_shared<LoadedTreeModel>(LoadedTreeModel::from_name(params.tree_params.name));
				tree_models.insert({params.tree_params.name, tree_model});
			}

			tree_model = tree_models.find(params.tree_params.name)->second;
		}

		// Create the scannable points
		std::vector<std::vector<SurfacePoint>> all_scannable_points;
		for (const auto &fruit_mesh: tree_model->meshes.fruit_meshes) {
			all_scannable_points.push_back(sample_points_on_mesh(rng, fruit_mesh, params.n_scannable_points_per_fruit));
		}

		const auto scaled_leaves = scale_leaves(tree_model->meshes, tree_model->root_points, params.tree_params.leaf_scale);

		return {
				.robot = robot,
				.tree_model = tree_model,
				.scaled_leaves = scaled_leaves,
				.scannable_points = all_scannable_points,
				.mesh_occlusion_model = std::make_shared<MeshOcclusionModel>(scaled_leaves, 0.0)
		};

	}

};

std::vector<PointScanEvalParameters> gen_eval_params() {
	SensorScalarParameters sensor_params {
			.maxViewDistance = INFINITY,
			.minViewDistance = 0.0,
			.fieldOfViewAngle = M_PI / 3.0,
			.maxScanAngle = M_PI / 3.0,
	};

	std::vector<TreeModelParameters> tree_params;

	for (const double leaf_scale: {0.0, 0.5, 1.0, 1.5, 2.0}) {

		TreeModelParameters params{
				.name = "appletree",
				.leaf_scale = leaf_scale,
				.fruit_subset = Unchanged{}
		};

		tree_params.push_back(params);

	}

	std::vector<PointScanEvalParameters> eval_params;

	for (const auto &tree_param: tree_params) {
		eval_params.push_back({
									  .tree_params = tree_param,
									  .sensor_params = sensor_params
							  });
	}

	return eval_params;
}

int main() {

	const auto& eval_params = gen_eval_params();

	const std::vector<OrbitPathParameters> orbits {
			{.parameters = CircularOrbitParameters{.radius = 1.0}},
			{.parameters = SphericalOscillationParameters{.radius = 1.0, .amplitude = 0.5, .cycles = 4}}
	};

	// Initialize a random number generator
	random_numbers::RandomNumberGenerator rng;

	// Create a robot model
	const auto &robot = experiments::createProceduralRobotModel();
	const robot_model::RobotModel::LinkId flying_base = robot.findLinkByName("flying_base");

	std::unordered_map<std::string, LoadedTreeModel> tree_models;

	Json::Value stats;

	EnvironmentInstanceCache environment_cache = {
			.robot = robot,
	};

	for (const auto &scenario_params: eval_params) {
		for (const auto& orbit: orbits){

			const auto &env = environment_cache.create_environment(scenario_params, rng);

			// Define the speed of interpolation
			double interpolation_speed = 0.01;

			Json::Value problem_stats;

			const RobotPath &path = parametricPathToRobotPath(robot,
															  env.tree_model->leaves_aabb.center(),
															  instantiatePath(env.tree_model->leaves_aabb.center(),
																			  orbit,
																			  env.tree_model->canopy_radius));

			Json::Value run;

			run["parameters"] = toJson(scenario_params);
			run["orbit"] = toJson(orbit);
			run["result"] = eval_path(path,
									  interpolation_speed,
									  env.scannable_points,
									  scenario_params.sensor_params,
									  env.mesh_occlusion_model);

			problem_stats.append(run);

		}

	}

	// Write the JSON object to a file
	Json::StreamWriterBuilder writer;
	std::ofstream file("../analysis/data/point_scanning.json");
	file << Json::writeString(writer, stats);

}