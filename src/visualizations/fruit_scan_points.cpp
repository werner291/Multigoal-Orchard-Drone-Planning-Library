// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <random_numbers/random_numbers.h>
#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/VtkLineSegmentVizualization.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../math/Triangle.h"
#include "../experiment_utils/surface_points.h"

using namespace mgodpl;

int main(int argc, char** argv) {

	auto tree_model = tree_meshes::loadTreeMeshes("appletree");

	// Grab the fruit mesh
	shape_msgs::msg::Mesh fruit_mesh = tree_model.fruit_meshes[0];

	random_numbers::RandomNumberGenerator rng;

	std::vector<SurfacePoint> fruit_points = sample_points_on_mesh(rng, fruit_mesh, 200);

	VtkLineSegmentsVisualization fruit_points_visualization(1,0,1);

	std::vector<std::pair<math::Vec3d, math::Vec3d>> fruit_lines;

	fruit_lines.reserve(fruit_points.size());
	for (const auto &fruit_point: fruit_points) {
		fruit_lines.push_back({fruit_point.position, fruit_point.position + fruit_point.normal * 0.05});
	}
	fruit_points_visualization.updateLine(fruit_lines);

	SimpleVtkViewer viewer;

	viewer.addMesh(tree_model.fruit_meshes[0], {1.0, 0.0, 0.0}, 1.0);

	viewer.addActor(fruit_points_visualization.getActor());

	viewer.start();

}