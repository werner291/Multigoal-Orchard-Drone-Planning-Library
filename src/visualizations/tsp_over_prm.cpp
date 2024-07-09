// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "../experiment_utils/default_colors.h"
#include "../visualization/visualization_function_macros.h"
#include "../visualization/robot_state.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../planning/RandomNumberGenerator.h"
#include "../planning/RobotState.h"
#include "../planning/state_tools.h"

using namespace mgodpl;

REGISTER_VISUALIZATION(tsp_over_prm) {
	auto tree_model = tree_meshes::loadTreeMeshes("appletree");
	viewer.addMesh(tree_model.trunk_mesh, WOOD_COLOR);

	// Load a robot model.
	auto robot = experiments::createProceduralRobotModel();

	// Create a random number generator.
	random_numbers::RandomNumberGenerator rng;

	// Create a PRM.
	struct Node {
		RobotState state;
		std::vector<size_t> neighbors;
	};

	std::vector<Node> nodes;

	const size_t max_samples = 100;

	for (size_t i = 0; i < max_samples; ++i) {
		auto state = generateUniformRandomState(robot, rng, 5.0, 10.0);
		auto robot_viz = vizualisation::vizualize_robot_state(viewer,
		                                                      robot,
		                                                      robot_model::forwardKinematics(
			                                                      robot,
			                                                      state.joint_values,
			                                                      0,
			                                                      state.base_tf));
		nodes.push_back({state, {}});
	}

	viewer.start();
}
