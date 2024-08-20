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

REGISTER_BENCHMARK(sampling_difficulty) {

	// Get all tree models:
	const auto& tree_names = tree_meshes::getTreeModelNames();

	RobotArmParameters params {
		.total_arm_length = 1.0,
		.joint_types = {JointType::HORIZONTAL},
		.add_spherical_wrist = false
	};

	// Create a robot model
	robot_model::RobotModel robot = createProceduralRobotModel(params);

	// Look up links:
	auto flying_base = robot.findLinkByName("flying_base");
	auto end_effector = robot.findLinkByName("end_effector");

	random_numbers::RandomNumberGenerator rng(42);

	// For each tree model name
	for (const auto &tree_model_name: mgodpl::tree_meshes::getTreeModelNames()) {
		// Load the tree meshes
		auto tree_mesh = mgodpl::tree_meshes::loadTreeMeshes(tree_model_name);

		math::AABBd tree_aabb = mesh_aabb(tree_mesh.trunk_mesh);

		std::cout << "Creating collision object for tree model " << tree_model_name << std::endl;

		// Create a collision object
		fcl::CollisionObjectd trunk_collision(mgodpl::fcl_utils::meshToFclBVH(tree_mesh.trunk_mesh));

		// Create a convex hull of the leaves of the tree
		auto tree_convex_hull = std::make_unique<mgodpl::cgal::CgalMeshData>(tree_mesh.leaves_mesh);

		CGAL::Side_of_triangle_mesh<mgodpl::cgal::Surface_mesh, mgodpl::cgal::K> inside(tree_convex_hull->convex_hull);

		double sample_h = std::max(tree_aabb.size().getX(), tree_aabb.size().getY()) / 2.0 + params.total_arm_length * 2.0;
		double sample_z = tree_aabb.min().getZ() + params.total_arm_length;

		std::cout << "Sampling for tree model " << tree_model_name << std::endl;

		results["results"][tree_model_name]["h_radius"] = sample_h;
		results["results"][tree_model_name]["z_radius"] = sample_z;

		// Sample 10,000 states.
		for (int i = 0; i < 10000; ++i) {
			// Sample a random state:
			RobotState state = generateUniformRandomState(robot, rng, sample_h, sample_z);

			bool collides = check_robot_collision(robot, trunk_collision, state);

			math::Vec3d base_center = state.base_tf.translation;

			double distance = sqrt(tree_convex_hull->tree.squared_distance(mgodpl::cgal::to_cgal_point(base_center)));

			bool inside_tree = inside(mgodpl::cgal::to_cgal_point(base_center)) == CGAL::ON_BOUNDED_SIDE;

			double signed_distance = inside_tree ? -distance : distance;

			// Record in results:
			results["results"][tree_model_name]["samples"]["signed_depth"].append(signed_distance);
			results["results"][tree_model_name]["samples"]["collides"].append(collides);

		}
	}

}

