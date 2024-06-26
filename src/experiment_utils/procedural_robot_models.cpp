// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/23/23.
//

#include "../planning/RobotModel.h"

#include "procedural_robot_models.h"
#include "mesh_from_dae.h"

namespace mgodpl {
	using namespace experiments;
	using namespace robot_model;

	/**
	 * @brief Creates a procedural robot model.
	 *
	 * This function creates a robot model with a flying base, a stick, and an end effector.
	 * The flying base is a box with a size of 0.8x0.8x0.2 and has a visual representation from a .dae file.
	 * The stick is a box with a size of 0.05xARM_LENGTHx0.05.
	 * The end effector is a link with no additional properties.
	 * Two joints are created to connect these parts: one revolute joint between the flying base and the stick, and one fixed joint between the stick and the end effector.
	 *
	 * @return RobotModel The created robot model.
	 */
	robot_model::RobotModel mgodpl::experiments::createProceduralRobotModel(const RobotArmParameters &parameters) {

		// The length of the stick in the robot model.
		double stick_length = parameters.arm_segment_length();

		// The robot model to be returned.
		RobotModel model;

		mgodpl::Mesh base_mesh = from_dae("../test_robots/meshes/drone.dae");
		Box base_box{.size = {0.8, 0.8, 0.2}};

		// The flying base of the robot model.
		RobotModel::LinkId flying_base = model.insertLink(
				{
						.name = "flying_base",
						.joints = {},
						.collision_geometry = {PositionedShape::untransformed(base_box)},
						.visual_geometry = {PositionedShape::untransformed(base_mesh)}
				});

		// The stick of the robot model.
		RobotModel::LinkId attach_to = flying_base;
		math::Transformd attachmentA = math::Transformd::fromTranslation({0, 0.2, 0});

		int name_counter = 1;

		for (const JointType &jt: parameters.joint_types) {
			const math::Vec3d axis = jt == JointType::HORIZONTAL ? math::Vec3d{1, 0, 0} : math::Vec3d{0, 0, 1};

			std::string joint_name = "stick_joint_" + std::to_string(name_counter++);

			math::Transformd stick_link_tf = math::Transformd::fromTranslation({0, -stick_length / 2, 0});
			RobotModel::LinkId link = model.insertLink(RobotModel::Link::namedBox("stick", {0.05, stick_length, 0.05}));

			model.insertJoint({
									  .name = joint_name,
									  .attachmentA = attachmentA,
									  .attachmentB = stick_link_tf,
									  .linkA = attach_to,
									  .linkB = link,
									  .type_specific = RobotModel::RevoluteJoint{
											  .axis = axis,
											  .min_angle = -M_PI,
											  .max_angle = M_PI
									  }
							  });

			attachmentA = stick_link_tf.inverse();
			attach_to = link;

		}

		// The end effector of the robot model.
		RobotModel::LinkId end_effector = model.insertLink({
																   .name = "end_effector",
																   .joints = {}
														   });

		// The joint connecting the stick and the end effector.
		model.insertJoint({
								  .name = "stick_end_attachment",
								  .attachmentA = attachmentA,
								  .attachmentB = math::Transformd::identity(),
								  .linkA = attach_to,
								  .linkB = end_effector,
								  .type_specific = RobotModel::FixedJoint{}
						  });

		// Return the created robot model.
		return model;
	}

	mgodpl::robot_model::RobotModel experiments::createProceduralRobotModel() {
		return createProceduralRobotModel({ARM_LENGTH, {JointType::HORIZONTAL}, false});
	}
}
