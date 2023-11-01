// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <moveit/robot_state/robot_state.h>
#include "../experiment_utils/load_robot_model.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/VtkRobotModel.h"

#include <moveit/robot_state/robot_state.h>

using namespace mgodpl;
using namespace visualization;
using namespace moveit::core;

int main() {

	const auto& robot = experiment_assets::loadRobotModel(1.0);

	RobotState rs(robot);
	rs.setToRandomPositions();

	SimpleVtkViewer viewer;

	visualization::VtkRobotModel robotModelViz(robot, rs, {0.5, 0.5, 0.5});
	viewer.addActorCollection(robotModelViz.getLinkActors());

	viewer.start();

	return 0;

}