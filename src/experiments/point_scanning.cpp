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

	auto scaled_leaves = scale_leaves(tree_model, leaf_roots, 0.2);
	viewer.addMesh(scaled_leaves, LEAF_COLOR);


	viewer.start();

    return 0;
}