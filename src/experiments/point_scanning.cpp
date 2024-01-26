// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/collision.h>
#include <vtkActor.h>

#include "../math/Transform.h"
#include "../planning/RobotModel.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../planning/ConvexHullSpace.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/VtkTriangleSetVisualization.h"
#include "../experiment_utils/default_colors.h"
#include "../experiment_utils/leaf_scaling.h"

using namespace mgodpl;

int main() {

	const auto& robot = experiments::createProceduralRobotModel();

	robot_model::RobotModel::LinkId flying_base = robot.findLinkByName("flying_base");
	robot_model::RobotModel::LinkId end_effector = robot.findLinkByName("end_effector");
	robot_model::RobotModel::LinkId stick = robot.findLinkByName("stick");

	const auto& tree_model = tree_meshes::loadTreeMeshes("appletree");

	SimpleVtkViewer viewer;

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

		leaf_triangles.clear();

		for (const auto& triangle : leaves_mesh.triangles)
		{
			leaf_triangles.push_back({
				 math::Vec3d {leaves_mesh.vertices[triangle.vertex_indices[0]].x, leaves_mesh.vertices[triangle.vertex_indices[0]].y, leaves_mesh.vertices[triangle.vertex_indices[0]].z},
				 math::Vec3d {leaves_mesh.vertices[triangle.vertex_indices[1]].x, leaves_mesh.vertices[triangle.vertex_indices[1]].y, leaves_mesh.vertices[triangle.vertex_indices[1]].z},
				 math::Vec3d {leaves_mesh.vertices[triangle.vertex_indices[2]].x, leaves_mesh.vertices[triangle.vertex_indices[2]].y, leaves_mesh.vertices[triangle.vertex_indices[2]].z}
		   });
		}

		leaves_visualization.updateTriangles(leaf_triangles);

		if (t > 6.0 * M_PI) {
			viewer.stop();
		}

	});

//	viewer.startRecording("leaf_scaling.ogv");

	viewer.start();

    return 0;
}