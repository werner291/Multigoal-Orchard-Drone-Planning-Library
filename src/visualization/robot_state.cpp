// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 2/12/24.
//

#include "robot_state.h"

mgodpl::vizualisation::RobotActors mgodpl::vizualisation::vizualize_robot_state(mgodpl::SimpleVtkViewer &viewer,
																				const mgodpl::robot_model::RobotModel &robot,
																				const mgodpl::robot_model::ForwardKinematicsResult &fk,
																				const mgodpl::math::Vec3d &color,
																				bool collision_only) {

	std::vector<vtkSmartPointer<vtkActor>> actors;

	for (size_t link_id = 0; link_id < robot.getLinks().size(); ++link_id) {
		auto link_tf = fk.forLink(link_id);

		// If it has visual shapes, add them.
		if (const auto &visual_geometry = robot.getLinks()[link_id].visual_geometry; !visual_geometry.empty() &&
																					 !collision_only) {
			for (const auto &shape: visual_geometry) {
				PositionedShape global{
						.shape = shape.shape,
						.transform = link_tf.then(shape.transform)
				};
				actors.push_back(viewer.addPositionedShape(global, color, 1.0));
			}
		} else {
			// Use the collision geometry as a fallback.
			for (const auto &shape: robot.getLinks()[link_id].collision_geometry) {
				PositionedShape global{
						.shape = shape.shape,
						.transform = link_tf.then(shape.transform)
				};
				actors.push_back(viewer.addPositionedShape(global, color, 1.0));
			}
		}
	}

	return RobotActors{.actors = actors};
}

void mgodpl::vizualisation::update_robot_state(const mgodpl::robot_model::RobotModel &robot,
											   const mgodpl::robot_model::ForwardKinematicsResult &fk,
											   mgodpl::vizualisation::RobotActors &actors) {

	auto actor_it = actors.actors.begin();

	for (size_t link_id = 0; link_id < robot.getLinks().size(); ++link_id) {
		auto link_tf = fk.forLink(link_id);

		// If it has visual shapes, add them.
		if (const auto &visual_geometry = robot.getLinks()[link_id].visual_geometry; !visual_geometry.empty()) {
			for (const auto &shape: visual_geometry) {
				PositionedShape global{
						.shape = shape.shape,
						.transform = link_tf.then(shape.transform)
				};
				SimpleVtkViewer::set_transform(global.transform, actor_it->Get());
				++actor_it;
			}
		} else {
			// Use the collision geometry as a fallback.
			for (const auto &shape: robot.getLinks()[link_id].collision_geometry) {
				PositionedShape global{
						.shape = shape.shape,
						.transform = link_tf.then(shape.transform)
				};
				SimpleVtkViewer::set_transform(global.transform, actor_it->Get());
				++actor_it;
			}
		}
	}
}
