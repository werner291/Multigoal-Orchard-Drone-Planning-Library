// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 2/12/24.
//

#include "scannable_points.h"

using namespace mgodpl;

VtkLineSegmentsVisualization createFruitLinesVisualization(const mgodpl::ScannablePoints &scannable_points) {
	VtkLineSegmentsVisualization fruit_points_visualization(1, 1, 1);

	std::vector<std::pair<math::Vec3d, math::Vec3d> > fruit_lines;
	fruit_lines.reserve(scannable_points.surface_points.size());
	for (const auto &[position, normal]: scannable_points.surface_points) {
		fruit_lines.emplace_back(position + normal * 0.01, position + normal * 0.02);
	}
	fruit_points_visualization.updateLine(fruit_lines);

	return fruit_points_visualization;
}

std::vector<mgodpl::math::Vec3d> generateVisualizationColors(const SeenPoints &ever_seen) {
	std::vector<math::Vec3d> vis_colors;
	for (const auto &v: ever_seen.ever_seen) {
		if (v) {
			vis_colors.emplace_back(0.0, 1.0, 0.0);
		} else {
			vis_colors.emplace_back(1.0, 0.0, 0.0);
		}
	}
	return vis_colors;
}

VtkLineSegmentsVisualization createFruitLinesVisualization(const std::vector<mgodpl::SurfacePoint> &scannable_points) {
	VtkLineSegmentsVisualization fruit_points_visualization(1, 1, 1);

	std::vector<std::pair<math::Vec3d, math::Vec3d> > fruit_lines;
	fruit_lines.reserve(scannable_points.size());
	for (const auto &[position, normal]: scannable_points) {
		fruit_lines.emplace_back(position, position + normal * 0.01);
	}
	fruit_points_visualization.updateLine(fruit_lines);

	return fruit_points_visualization;
}

VtkLineSegmentsVisualization visualize(mgodpl::SimpleVtkViewer &viewer,
                                       const ScannablePoints &scannable_points,
                                       const SeenPoints &initial_seen_status) {
	return visualize(viewer, scannable_points.surface_points, initial_seen_status);
}

VtkLineSegmentsVisualization visualize(mgodpl::SimpleVtkViewer &viewer,
                                       const std::vector<mgodpl::SurfacePoint> &scannable_points,
                                       const SeenPoints &initial_seen_status) {
	// Create the fruit points visualization
	VtkLineSegmentsVisualization fruit_points_visualization = createFruitLinesVisualization(scannable_points);
	viewer.addActor(fruit_points_visualization.getActor());
	// Set the color:
	fruit_points_visualization.setColors(generateVisualizationColors(initial_seen_status));
	return fruit_points_visualization;
}

void update_visualization(const SeenPoints &ever_seen, VtkLineSegmentsVisualization &fruit_points_visualization) {
	fruit_points_visualization.setColors(generateVisualizationColors(ever_seen));
}
