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

using namespace mgodpl;

Json::Value eval_path(const RobotPath &path,
 					  double interpolation_speed,
					  const size_t NUM_POINTS,
					  const std::vector<ScannablePoints> &all_scannable_points) {// Create a JSON object


	// Define the current position on the path
	PathPoint path_point = {0, 0.0};

	std::vector<SeenPoints> ever_seen;
	ever_seen.reserve(all_scannable_points.size());
	for (const auto &scannable_points: all_scannable_points) {
		ever_seen.push_back(SeenPoints::create_all_unseen(scannable_points));
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
		frame_stats["rdist"] = (angular_distance(interpolated_state.base_tf.orientation, last_state.base_tf.orientation));
		frame_stats["jd"] = Json::arrayValue;
		for (size_t i = 0; i < interpolated_state.joint_values.size(); ++i) {
			frame_stats["jd"].append(std::abs(interpolated_state.joint_values[i] - last_state.joint_values[i]));
		}

		// Get the position of the robot's end effector
		const auto &end_effector_position = interpolated_state.base_tf.translation;

		// Update the visibility of the scannable points
		for (size_t fruit_i = 0; fruit_i < all_scannable_points.size(); ++fruit_i) {
			for (size_t i = 0; i < all_scannable_points[fruit_i].surface_points.size(); ++i) {
				if (!ever_seen[fruit_i].ever_seen[i] &&
					is_visible(all_scannable_points[fruit_i], i, end_effector_position)) {
					ever_seen[fruit_i].ever_seen[i] = true;
				}
			}
		}

		size_t seen_total = 0;
		size_t unique_total = 0;

		for (size_t fruit_i = 0; fruit_i < all_scannable_points.size(); ++fruit_i) {
			seen_total += std::count(ever_seen[fruit_i].ever_seen.begin(), ever_seen[fruit_i].ever_seen.end(), true);
			unique_total += std::any_of(ever_seen[fruit_i].ever_seen.begin(), ever_seen[fruit_i].ever_seen.end(), [](bool b) { return b; });
		}

		// Calculate metrics
		double percent_surface_points_seen = 100.0 * static_cast<double>(seen_total) / static_cast<double>(NUM_POINTS * all_scannable_points.size());
		// Round it:
		percent_surface_points_seen = round(percent_surface_points_seen * 100) / 100;

		// Add the metrics to the JSON object
		frame_stats["pts_seen"] = percent_surface_points_seen;
		frame_stats["n_unique"] = unique_total;

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

int main() {
    // Load the tree meshes
    auto tree_model = tree_meshes::loadTreeMeshes("appletree");

    // Initialize a random number generator
    random_numbers::RandomNumberGenerator rng;

    // Create a robot model
    const auto &robot = experiments::createProceduralRobotModel();
    const robot_model::RobotModel::LinkId flying_base = robot.findLinkByName("flying_base");

    // Define the center of the tree
    const auto leaves_aabb = mesh_aabb(tree_model.leaves_mesh);
    math::Vec3d tree_center = leaves_aabb.center();
    double tree_radius = leaves_aabb.size().norm() / 2.0;

	// Define the speed of interpolation
    double interpolation_speed = 0.01;

    // Define constants for the scannable points
    const size_t NUM_POINTS = 200;
    const double MAX_DISTANCE = INFINITY;
    const double MIN_DISTANCE = 0;
    const double MAX_ANGLE = M_PI / 3.0;

	Json::Value stats;

	auto root_points = leaf_root_points(tree_model);

	for (const double leaf_scale : {0.0,0.5,1.0,1.5,2.0}) {

		auto leaves = scale_leaves(tree_model, root_points, leaf_scale);

		auto mesh_occlusion_model = std::make_shared<MeshOcclusionModel>(leaves, 0.0);

		// Create the scannable points
		std::vector<ScannablePoints> all_scannable_points;
		for (const auto &fruit_mesh: tree_model.fruit_meshes) {
			all_scannable_points.push_back(createScannablePoints(
					rng,
					fruit_mesh,
					NUM_POINTS,
					MAX_DISTANCE,
					MIN_DISTANCE,
					MAX_ANGLE,
					mesh_occlusion_model
			));
		}

		Json::Value problem_stats;

		{    // Get the first JsonMeta<ParametricPath> object
			ParametricPath first_orbit = fixed_radius_equatorial_orbit(tree_center, tree_radius);
			RobotPath path = parametricPathToRobotPath(robot, tree_center, first_orbit);

			problem_stats["equatorial_orbit"] = eval_path(path,
														  interpolation_speed,
														  NUM_POINTS,
														  all_scannable_points);
		}

		{
			ParametricPath first_orbit = latitude_oscillation_path(tree_center, tree_radius, 0.5, 4);
			RobotPath path = parametricPathToRobotPath(robot, tree_center, first_orbit);

			problem_stats["latitude_oscillation"] = eval_path(path,
															  interpolation_speed,
															  NUM_POINTS,
															  all_scannable_points);
		}

		stats["leaf_scale_" + std::to_string(leaf_scale)] = problem_stats;
	}

	// Write the JSON object to a file
	Json::StreamWriterBuilder writer;
	std::ofstream file("../analysis/data/point_scanning.json");
	file << Json::writeString(writer, stats);

}