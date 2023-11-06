// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/6/23.
//

#ifndef MGODPL_VTKENDANDBASETRACEVISUALIZATION_H
#define MGODPL_VTKENDANDBASETRACEVISUALIZATION_H

#include "../planning/moveit_forward_declarations.h"

namespace mgodpl::moveit_facade {
	struct JointSpacePath;
}

#include "VtkLineSegmentVizualization.h"
#include "VtkPolyLineVisualization.h"

namespace mgodpl::visualization {
	class VtkEndAndBaseTraceVisualization {

		const moveit::core::RobotModelConstPtr robotModel;
	public:

		VtkEndAndBaseTraceVisualization(const moveit::core::RobotModelConstPtr &robotModel);

		void updateLine(const moveit_facade::JointSpacePath &path);

		VtkLineSegmentsVisualization barTrace;
		VtkPolyLineVisualization endTrace;
		VtkPolyLineVisualization baseTrace;

	};
}


#endif //MGODPL_VTKENDANDBASETRACEVISUALIZATION_H
