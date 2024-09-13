// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.



#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/declarative_environment.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../planning/RandomNumberGenerator.h"
#include "../planning/RobotModel.h"
#include "../planning/RobotPath.h"
#include "../planning/fcl_utils.h"
#include "../planning/state_tools.h"
#include "../planning/local_optimization.h"
#include "../planning/tsp_over_prm.h"
#include "../visualization/Throttle.h"
#include "../visualization/VtkTriangleSetVisualization.h"
#include "../visualization/declarative.h"
#include "../visualization/ladder_trace.h"
#include "../visualization/robot_state.h"
#include "../visualization/visualization_function_macros.h"


using namespace mgodpl;

const declarative::TreeModelParameters DEFAULT_APPLE_TREE = {
		.name = "appletree",
		.leaf_scale = 1.0,
		.fruit_subset = declarative::Unchanged{},
		.seed = 42
};

// PRMGraph make_prm() {
// 	PRMGraph graph;
//
// 	// A vector of all the infrastructure nodes. (TODO: make this a spatial index instead.)
// 	ompl::NearestNeighborsGNAT<std::pair<RobotState, PRMGraph::vertex_descriptor> > spatial_index;
//
// 	return graph;
// }

REGISTER_VISUALIZATION(shortcutting) {
	// Create a random number generator.
	random_numbers::RandomNumberGenerator rng(42);

	// // Load the default tree model.
	// auto tree = instantiate_tree_model(DEFAULT_APPLE_TREE, rng);
	//
	// // Add the tree to the visualization.
	// visualization::visualize(viewer, tree);
	//
	// // Create a collision object BVH for the tree trunk.
	// fcl::CollisionObjectd tree_collision(fcl_utils::meshToFclBVH(tree.tree_model->meshes.trunk_mesh));

	// Load a robot model.
	auto robot = experiments::createProceduralRobotModel();

	RobotPath path;
	for (size_t i = 0; i < 10; i++) {
		path.append(generateUniformRandomState(robot, rng, 5, 10));
	}

	// compute FK.
	auto fk1 = forwardKinematics(robot, path.states[0].joint_values, 0, path.states[0].base_tf);

	auto robot_viz = vizualisation::vizualize_robot_state(viewer, robot, fk1);

	size_t max_repeats = 1;

	visualization::visualize_ladder_trace(robot, path, viewer);

	visualization::Throttle throttle;

	viewer.addTimerCallback([&]() {
		static double t = 0.0;
		t += 0.02;

		if (std::floor(t) >= path.states.size() - 1) {
			t = 0.0;

			// Try shortcutting:
			// tryShortcuttingRandomly(robot, path);
		}

		auto st = interpolate(PathPoint{(size_t) std::floor(t), std::fmod(t, 1.0)}, path);

		auto fk = forwardKinematics(robot, st.joint_values, 0, st.base_tf);

		// update.
		vizualisation::update_robot_state(robot, fk, robot_viz);
	});

	viewer.setCameraTransform({0.0, 20.0, 20.0}, {0.0, 0.0, 5.0});

	viewer.start();
}
