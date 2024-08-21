// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "ladder_trace.h"
#include "../experiment_utils/tracing.h"
#include "VtkPolyLineVisualization.h"
#include "VtkLineSegmentVizualization.h"

void mgodpl::visualization::visualize_ladder_trace(const mgodpl::robot_model::RobotModel &robot,
												   const mgodpl::RobotPath &final_path,
												   mgodpl::SimpleVtkViewer &viewer) {

	const auto &end_effector_positions = link_trace(robot, final_path, robot.findLinkByName("end_effector"));
	const auto &base_positions = link_trace(robot, final_path, robot.findLinkByName("flying_base"));
	std::vector<std::pair<math::Vec3d, math::Vec3d>> connections_data;
	for (size_t i = 0; i < final_path.states.size(); ++i) {
		connections_data.push_back({end_effector_positions[i], base_positions[i]});
	}

	VtkPolyLineVisualization end_effector_trace(1, 0, 1);
	viewer.addActor(end_effector_trace.getActor());
	end_effector_trace.updateLine(end_effector_positions);

	VtkPolyLineVisualization base_trace(1, 1, 0);
	viewer.addActor(base_trace.getActor());
	base_trace.updateLine(base_positions);

	VtkLineSegmentsVisualization connections(0.2, 1, 0.2);
	viewer.addActor(connections.getActor());
	connections.updateLine(connections_data);

}
