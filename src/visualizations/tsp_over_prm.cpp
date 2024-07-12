// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <vtkRenderer.h>
#include <fcl/narrowphase/collision_object.h>

#include "../experiment_utils/default_colors.h"
#include "../visualization/visualization_function_macros.h"
#include "../visualization/robot_state.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../planning/RandomNumberGenerator.h"
#include "../planning/RobotState.h"
#include "../planning/state_tools.h"
#include "../planning/vptree.hpp"
#include "../planning/fcl_utils.h"
#include "../visualization/VtkLineSegmentVizualization.h"
#include <fcl/narrowphase/collision.h>

#include "../planning/collision_detection.h"

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
	const size_t n_neighbours = 5;

	VtkLineSegmentsVisualization prm_edges(1, 0, 1);
	std::vector<std::pair<math::Vec3d, math::Vec3d> > edges;
	viewer.addActor(prm_edges.getActor());

	int frames_until_sample = 0;

	// Allocate a BVH convex_hull for the tree trunk.
	auto tree_collision = fcl_utils::treeMeshesToFclCollisionObject(tree_model);

	std::optional<vizualisation::RobotActors> sample_viz;

	viewer.addTimerCallback([&]() {
		if (frames_until_sample > 0) {
			--frames_until_sample;
			return;
		}
		frames_until_sample = 1;

		if (sample_viz) {
			for (auto vtk_actor: sample_viz->actors) {
				viewer.viewerRenderer->RemoveActor(vtk_actor);
			}
		}

		// Skip if we're at the maximum number of samples.
		if (nodes.size() >= max_samples) {
			viewer.stop();
			return;
		}

		auto state = generateUniformRandomState(robot, rng, 5.0, 10.0);

		bool collides = check_robot_collision(robot, tree_collision, state);

		math::Vec3d color = collides
			                    ? math::Vec3d{1.0, 0.0, 0.0}
			                    : // Red if it collides.
			                    math::Vec3d{0.0, 1.0, 0.0}; // Green if it doesn't.

		sample_viz = vizualisation::vizualize_robot_state(viewer,
		                                                  robot,
		                                                  robot_model::forwardKinematics(
			                                                  robot,
			                                                  state.joint_values,
			                                                  0,
			                                                  state.base_tf),
		                                                  color);

		// If this sample collides, don't add it to the roadmap.
		if (collides) {
			return;
		}

		// Find the k nearest neighbors. (Brute-force; should migrate to a VP-tree when I can.)

		// Keep the distances and the indices.
		std::vector<std::pair<double, size_t> > distances;
		distances.reserve(n_neighbours + 1);

		// Iterate over all nodes.
		for (size_t j = 0; j < nodes.size(); ++j) {
			bool inserted = false;

			// Iterate over all neighbors found so far.
			for (size_t k = 0; k < distances.size(); ++k) {
				// If the distance is smaller than the current distance, insert it.
				if (distances[k].first > equal_weights_distance(nodes[j].state, state)) {
					distances.insert(distances.begin() + k, {equal_weights_distance(nodes[j].state, state), j});
					inserted = true;
					// If it's now larger than n_neighbours, remove the last element.
					if (distances.size() > n_neighbours) {
						distances.pop_back();
					}
					break;
				}
			}

			if (!inserted && distances.size() < n_neighbours) {
				distances.push_back({equal_weights_distance(nodes[j].state, state), j});
			}
		}

		// Extract just the indices.
		std::vector<size_t> neighbors;
		neighbors.reserve(n_neighbours);
		for (const auto &d: distances) {
			neighbors.push_back(d.second);

			// In the visualization, draw the edge between the base links:
			edges.push_back({nodes[d.second].state.base_tf.translation, state.base_tf.translation});
		}

		nodes.push_back({state, neighbors});

		prm_edges.updateLine(edges);
	});

	viewer.start();
}
