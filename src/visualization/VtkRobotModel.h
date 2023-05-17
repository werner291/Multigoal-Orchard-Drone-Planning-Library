//
// Created by werner on 30-9-22.
//

#ifndef NEW_PLANNERS_VTKROBOTMODEL_H
#define NEW_PLANNERS_VTKROBOTMODEL_H

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <vtkActorCollection.h>
#include <vtkNew.h>

class VtkRobotmodel {

	moveit::core::RobotModelConstPtr robotModel;

	// One actor per link in the robot.
	vtkNew<vtkActorCollection> link_actors;
public:
	[[nodiscard]] vtkNew<vtkActorCollection> &getLinkActors();

	explicit VtkRobotmodel(const moveit::core::RobotModelConstPtr &robot_model,
						   const moveit::core::RobotState &initial_state,
						   const Eigen::Vector3d &rgb = {0.5, 0.5, 0.5});

	void applyState(const moveit::core::RobotState& st);



};



#endif //NEW_PLANNERS_VTKROBOTMODEL_H
