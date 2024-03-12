// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/12/24.
//

#include "declarative.h"
#include "../experiment_utils/default_colors.h"
#include "../experiment_utils/tree_models.h"
#include "VtkLineSegmentVizualization.h"
#include "scannable_points.h"
#include "../experiment_utils/scan_paths.h"

using namespace mgodpl;

void mgodpl::visualization::visualize(mgodpl::SimpleVtkViewer &viewer,
									  const declarative::FruitModels &fruitModels) {

	if (auto spheres = std::get_if<std::vector<declarative::SphericalFruit>>(&fruitModels)) {
		for (const auto &sphere: *spheres) {
			viewer.addSphere(sphere.radius, sphere.center, FRUIT_COLOR);
		}
	} else if (auto meshes = std::get_if<std::vector<declarative::MeshFruit>>(&fruitModels)) {
		for (const auto &mesh: *meshes) {
			viewer.addMesh(mesh.mesh, FRUIT_COLOR);
		}
	}
}

void mgodpl::visualization::visualize(mgodpl::SimpleVtkViewer &viewer,
									  const mgodpl::declarative::FullTreeModel &model) {
	viewer.addMesh(model.tree_model->meshes.trunk_mesh, WOOD_COLOR);
	viewer.addMesh(model.scaled_leaves, LEAF_COLOR);
	visualize(viewer, model.fruit_models);
}

void mgodpl::visualization::visualize(mgodpl::SimpleVtkViewer &viewer,
									  const std::vector<std::vector<SurfacePoint>> &scannablePoints) {

	// Create the fruit points visualization
	std::vector<VtkLineSegmentsVisualization> fruit_points_visualizations;
	for (const auto &scannable_points: scannablePoints) {
		fruit_points_visualizations.push_back(createFruitLinesVisualization(scannable_points));
		viewer.addActor(fruit_points_visualizations.back().getActor());
	}
}

void visualization::visualize(SimpleVtkViewer &viewer,
							  const mgodpl::ParametricPath &path,
							  int n_points,
							  const math::Vec3d &color) {

	VtkPolyLineVisualization path_viz(color[0], color[1], color[2]);

	std::vector<math::Vec3d> points;
	for (int i = 0; i < n_points; i++) {
		double t = i / (n_points - 1.0);
		points.push_back(path(t));
	}
	path_viz.updateLine(points);
	viewer.addActor(path_viz.getActor());
}
