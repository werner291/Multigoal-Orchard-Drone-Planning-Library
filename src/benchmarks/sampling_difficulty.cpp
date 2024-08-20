// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 8/7/24.
//

#include "benchmark_function_macros.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../planning/RobotModel.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../planning/fcl_utils.h"
#include "../planning/cgal_chull_shortest_paths.h"
#include "../planning/RobotState.h"
#include "../planning/RandomNumberGenerator.h"
#include "../planning/state_tools.h"
#include "../planning/collision_detection.h"

#include <CGAL/Side_of_triangle_mesh.h>

using namespace mgodpl;
using namespace experiments;

/**
 * @brief Register a benchmark for sampling difficulty.
 *
 * This function registers a benchmark that measures the difficulty of sampling robot states in different tree models.
 * The difficulty is measured by the number of collisions and the signed depth of the robot's base center in the tree.
 *
 * The benchmark is registered under the name "sampling_difficulty".
 */
REGISTER_BENCHMARK(sampling_difficulty) {

	// Get all tree models:
	const auto& tree_names = tree_meshes::getTreeModelNames();

	// Define the parameters for the robot arm
	RobotArmParameters params {
		.total_arm_length = 1.0,
		.joint_types = {JointType::HORIZONTAL},
		.add_spherical_wrist = false
	};

	// Create a robot model with the defined parameters
	robot_model::RobotModel robot = createProceduralRobotModel(params);

	// Look up links:
	auto flying_base = robot.findLinkByName("flying_base");
	auto end_effector = robot.findLinkByName("end_effector");

	// Initialize a random number generator
	random_numbers::RandomNumberGenerator rng(42);

	// For each tree model name
	for (const auto &tree_model_name: mgodpl::tree_meshes::getTreeModelNames()) {
		// Load the tree meshes
		auto tree_mesh = mgodpl::tree_meshes::loadTreeMeshes(tree_model_name);

		// Compute the axis-aligned bounding box of the tree trunk
		math::AABBd tree_aabb = mesh_aabb(tree_mesh.trunk_mesh);

		std::cout << "Creating collision object for tree model " << tree_model_name << std::endl;

		// Create a collision object for the tree trunk
		fcl::CollisionObjectd trunk_collision(mgodpl::fcl_utils::meshToFclBVH(tree_mesh.trunk_mesh));

		// Create a convex hull of the leaves of the tree
		auto tree_convex_hull = std::make_unique<mgodpl::cgal::CgalMeshData>(tree_mesh.leaves_mesh);

		// Initialize a Side_of_triangle_mesh object for checking whether a point is inside the convex hull
		CGAL::Side_of_triangle_mesh<mgodpl::cgal::Surface_mesh, mgodpl::cgal::K> inside(tree_convex_hull->convex_hull);

		// Compute the sampling radii
		double sample_h = std::max(tree_aabb.size().getX(), tree_aabb.size().getY()) / 2.0 + params.total_arm_length * 2.0;
		double sample_z = tree_aabb.min().getZ() + params.total_arm_length;

		std::cout << "Sampling for tree model " << tree_model_name << std::endl;

		// Record the sampling radii in the results
		results[tree_model_name]["h_radius"] = sample_h;
		results[tree_model_name]["z_radius"] = sample_z;

		// Sample 10,000 states.
		for (int i = 0; i < 10000; ++i) {
			// Sample a random state:
			RobotState state = generateUniformRandomState(robot, rng, sample_h, sample_z);

			// Check whether the robot collides with the tree trunk
			bool collides = check_robot_collision(robot, trunk_collision, state);

			// Compute the center of the robot's base
			math::Vec3d base_center = state.base_tf.translation;

			// Compute the distance from the base center to the convex hull
			double distance = sqrt(tree_convex_hull->tree.squared_distance(mgodpl::cgal::to_cgal_point(base_center)));

			// Check whether the base center is inside the convex hull
			bool inside_tree = inside(mgodpl::cgal::to_cgal_point(base_center)) == CGAL::ON_BOUNDED_SIDE;

			// Compute the signed distance (negative if inside the convex hull, positive otherwise)
			double signed_distance = inside_tree ? -distance : distance;

			Json::Value sample_result;
			sample_result["base_center"][0] = base_center.getX();
			sample_result["base_center"][1] = base_center.getY();
			sample_result["base_center"][2] = base_center.getZ();
			sample_result["signed_depth"] = signed_distance;
			sample_result["collides"] = collides;

			// Record the base center, signed distance, and collision status in the results
			results[tree_model_name]["samples"].append(sample_result);

		}
	}

}

