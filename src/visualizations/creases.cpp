// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/visualization_function_macros.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../planning/cgal_chull_shortest_paths.h"
#include "../visualization/VtkTriangleSetVisualization.h"

using namespace mgodpl;

REGISTER_VISUALIZATION(creases)
{
	// Load the tree meshes
	auto tree_model = tree_meshes::loadTreeMeshes("appletree");

	viewer.addMesh(tree_model.trunk_mesh, math::Vec3d(0.5,0.3,0.1));

	const bool LOOP_ANIMATION = false;

	auto mesh = cgal::cgal_convex_hull_around_leaves(tree_model.trunk_mesh);

	VtkTriangleSetVisualization chull_viz(0.8,0.8,0.8,0.5);



	// Add a timer callback to the viewer
	viewer.addTimerCallback([&](){

							});

	// Start the viewer
	viewer.start();
}
