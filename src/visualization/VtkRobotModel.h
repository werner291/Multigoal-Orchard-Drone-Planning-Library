// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 30-9-22.
//

#ifndef NEW_PLANNERS_VTKROBOTMODEL_H
#define NEW_PLANNERS_VTKROBOTMODEL_H

#include <memory>

#include <vtkActorCollection.h>
#include <vtkNew.h>

#include "../math/Vec3.h"

namespace moveit::core {
	class RobotModel;
	class RobotState;
	using RobotModelConstPtr = std::shared_ptr<const RobotModel>;
}

namespace mgodpl::visualization {

	class VtkRobotModel {

		moveit::core::RobotModelConstPtr robotModel;

		// One actor per link in the robot.
		vtkNew<vtkActorCollection> link_actors;
	public:
		[[nodiscard]] vtkNew<vtkActorCollection> &getLinkActors();

		explicit VtkRobotModel(moveit::core::RobotModelConstPtr robot_model,
							   const moveit::core::RobotState &initial_state,
							   const math::Vec3d &rgb);

		void applyState(const moveit::core::RobotState &st);

		void generateLinkActors(const math::Vec3d &rgb);
	};
}

#endif //NEW_PLANNERS_VTKROBOTMODEL_H
