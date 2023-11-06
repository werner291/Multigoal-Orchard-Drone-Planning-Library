//
// Created by werner on 11/6/23.
//

#include "VtkEndAndBaseTraceVisualization.h"

#include "../planning/JointSpacePath.h"

void
mgodpl::visualization::VtkEndAndBaseTraceVisualization::updateLine(const mgodpl::moveit_facade::JointSpacePath &path) {

	std::vector<std::pair<mgodpl::math::Vec3d, mgodpl::math::Vec3d>> barPoints;
	std::vector<mgodpl::math::Vec3d> endPoints;
	std::vector<mgodpl::math::Vec3d> basePoints;

	for (const auto &state : path.path) {
		const auto &endEffectorPose = moveit_facade::computeEndEffectorPosition(*robotModel, state);
		const auto &basePose = moveit_facade::computeLinkEffectorPosition(*robotModel, state, "base_link");

		endPoints.push_back(endEffectorPose);
		basePoints.push_back(basePose);
		barPoints.emplace_back(endEffectorPose, basePose);
	}

	barTrace.updateLine(barPoints);
	endTrace.updateLine(endPoints);
	baseTrace.updateLine(basePoints);

}

mgodpl::visualization::VtkEndAndBaseTraceVisualization::VtkEndAndBaseTraceVisualization(const moveit::core::RobotModelConstPtr &robotModel)
	: robotModel(robotModel), barTrace(1.0,1.0,0.0), endTrace(1.0,0.0,0.0), baseTrace(0.0,0.0,1.0)
{

}
