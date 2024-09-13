// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 2/12/24.
//

#ifndef MGODPL_ROBOT_STATE_VIS_H
#define MGODPL_ROBOT_STATE_VIS_H

#include "vtk_declarations.h"
#include "SimpleVtkViewer.h"
#include "../planning/RobotModel.h"

namespace mgodpl::vizualisation {

	struct RobotActors {
		std::vector<vtkSmartPointer<vtkActor>> actors;
	};

	RobotActors vizualize_robot_state(SimpleVtkViewer &viewer,
									  const robot_model::RobotModel &robot,
									  const robot_model::ForwardKinematicsResult &fk,
									  const math::Vec3d &color = {0.8, 0.8, 0.8},
									  bool collision_only = false);

	void update_robot_state(const robot_model::RobotModel &robot,
							const robot_model::ForwardKinematicsResult &fk,
							RobotActors &actors);
}

#endif //MGODPL_ROBOT_STATE_VIS_H
