// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <moveit/robot_state/robot_state.h>

#include "../experiment_utils/load_robot_model.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/VtkRobotModel.h"
#include "../experiment_utils/moveit_state_tools.h"
#include "../experiment_utils/TreeMeshes.h"

#include <moveit/robot_state/robot_state.h>

using namespace mgodpl;
using namespace visualization;
using namespace moveit::core;
using namespace experiments;

int main() {

	// TODO: Reminder, write down the idea of the snake path!

	const auto& robot = experiment_assets::loadRobotModel(1.0);

	const auto& tree_model = tree_meshes::loadTreeMeshes("appletree");

	RobotState rs = randomStateOutsideTree(robot, 0);

	SimpleVtkViewer viewer;

	visualization::VtkRobotModel robotModelViz(robot, rs, {0.5, 0.5, 0.5});
	viewer.addActorCollection(robotModelViz.getLinkActors());

	int seed = 0;

	viewer.addTimerCallback([&]() {
		rs = randomStateOutsideTree(robot, seed++);
		robotModelViz.applyState(rs);
	});

	viewer.addMesh(tree_model.trunk_mesh, {0.5, 0.3, 0.1});
	viewer.addMesh(tree_model.leaves_mesh, {0.1, 0.5, 0.1});
	for (const auto& fruit : tree_model.fruit_meshes) viewer.addMesh(fruit, {0.5, 0.1, 0.1});

	viewer.start();

	return 0;

}