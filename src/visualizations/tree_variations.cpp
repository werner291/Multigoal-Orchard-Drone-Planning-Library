// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <vtkActor.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>

#include "../math/Transform.h"
#include "../planning/RobotModel.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../planning/ConvexHullSpace.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/VtkTriangleSetVisualization.h"
#include "../experiment_utils/default_colors.h"
#include "../experiment_utils/leaf_scaling.h"
#include "../visualization/visualization_function_macros.h"
#include "../experiment_utils/surface_points.h"
#include "../experiment_utils/procedural_fruit_placement.h"

using namespace mgodpl;

REGISTER_VISUALIZATION(leaf_density_rescaling) {
	auto tree_model = tree_meshes::loadTreeMeshes("appletree");

	viewer.addMesh(tree_model.trunk_mesh, WOOD_COLOR);

	auto leaf_roots = leaf_root_points(tree_model);

	VtkTriangleSetVisualization leaves_visualization(LEAF_COLOR[0], LEAF_COLOR[1], LEAF_COLOR[2], 1);

	std::vector<std::array<math::Vec3d, 3>> leaf_triangles;
	viewer.addActor(leaves_visualization.getActor());
	viewer.setCameraTransform({8, 0, 2}, {0, 0, 2});

	double t = 0.0;

	viewer.addTimerCallback([&]() {
		t += 0.1;
		auto leaves_mesh = scale_leaves(tree_model, leaf_roots, 1.0 + 0.5 * std::sin(t));

		leaf_triangles = triangles_from_mesh(leaves_mesh);

		leaves_visualization.updateTriangles(leaf_triangles);

		if (t > 3.0 * M_PI) {
			viewer.stop();
		}

	});

	viewer.start();
}

REGISTER_VISUALIZATION(procedural_fruit) {

	auto tree_model = tree_meshes::loadTreeMeshes("appletree");

	viewer.addMesh(tree_model.trunk_mesh, WOOD_COLOR);

	size_t num_fruits = 100;

	random_numbers::RandomNumberGenerator rng(42);

	std::vector<vtkSmartPointer<vtkActor>> fruit_actors;

	double change_countdown = 0.0;

	size_t repeat = 10;

	viewer.addTimerCallback([&]() {
		change_countdown -= 0.1;
		if (change_countdown < 0) {
			change_countdown = 2.0;
			for (auto &actor: fruit_actors) {
				viewer.viewerRenderer->RemoveActor(actor);
			}
			const auto &fruit_locations = generate_fruit_locations(tree_model, num_fruits, rng);
			fruit_actors.clear();
			for (const auto &pt: fruit_locations) {
				fruit_actors.push_back(viewer.addSphere(FRUIT_RADIUS, pt, {1, 0, 0}, 1.0));
			}
			if (repeat > 0) {
				repeat--;
			} else {
				viewer.stop();
			}
		}
	});

	viewer.setCameraTransform({6, 0, 2}, {0, 0, 2});

	viewer.start();
}


REGISTER_VISUALIZATION(procedural_fruit_clusters) {
	auto tree_model = tree_meshes::loadTreeMeshes("appletree");

	viewer.addMesh(tree_model.trunk_mesh, WOOD_COLOR);

	auto leaf_roots = leaf_root_points(tree_model);

	size_t num_fruits = 100;

	random_numbers::RandomNumberGenerator rng(42);

	std::vector<vtkSmartPointer<vtkActor>> fruit_actors;

	double change_countdown = 2.0;

	size_t repeat = 10;

	viewer.addTimerCallback([&]() {
		change_countdown -= 0.1;
		if (change_countdown < 0) {
			change_countdown = 2.0;
			for (auto &actor: fruit_actors) {
				viewer.viewerRenderer->RemoveActor(actor);
			}
			const auto &fruit_locations = generate_fruit_clusters(tree_model, num_fruits, 1, 5, true, rng);
			fruit_actors.clear();
			for (const auto &pt: fruit_locations) {
				fruit_actors.push_back(viewer.addSphere(FRUIT_RADIUS, pt, {1, 0, 0}, 1.0));
			}
			if (repeat > 0) {
				repeat--;
			} else {
				viewer.stop();
			}
		}


	});

	viewer.setCameraTransform({6, 0, 2}, {0, 0, 2});

	viewer.start();
}
