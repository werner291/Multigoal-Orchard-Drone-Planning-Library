// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/23/23.
//

#include "../planning/RobotModel.h"

#include "procedural_robot_models.h"

#include <sstream>

#include "mesh_from_dae.h"

namespace mgodpl {
	using namespace experiments;
	using namespace robot_model;

	std::string RobotArmParameters::short_designator() const {
		std::stringstream designator;
		designator << total_arm_length << "J";
		for (const auto &joint_type: joint_types) {
			designator << (joint_type == HORIZONTAL ? "H" : "V");
		}
		if (add_spherical_wrist) {
			designator << "S";
		}
		return designator.str();
	}

	std::vector<mgodpl::experiments::RobotArmParameters> experiments::generateRobotArmParameters(
			const RobotArmMetaParameters &meta_params) {
		std::vector<mgodpl::experiments::RobotArmParameters> robot_arm_parameters;

		for (const auto &arm_length: meta_params.arm_lengths) {
			for (size_t i = 0; i < meta_params.max_links; ++i) {
				std::vector<std::vector<mgodpl::experiments::JointType> > joint_types;

				if (meta_params.include_all_vertical) {
					// All vertical:
					joint_types.emplace_back(i, mgodpl::experiments::JointType::VERTICAL);
				}

				if (meta_params.include_all_horizontal) {
					// All horizontal:
					joint_types.emplace_back(i, mgodpl::experiments::JointType::HORIZONTAL);
				}

				if (meta_params.include_alternating_horizontal_vertical) {
					// Alternating horizontal/vertical:
					std::vector<mgodpl::experiments::JointType> alternating_joint_types;
					for (size_t j = 0; j < i; ++j) {
						alternating_joint_types.push_back(j % 2 == 0
														  ? mgodpl::experiments::JointType::HORIZONTAL
														  : mgodpl::experiments::JointType::VERTICAL);
					}
					joint_types.push_back(alternating_joint_types);
				}

				for (const auto &joint_type: joint_types) {
					robot_arm_parameters.push_back(mgodpl::experiments::RobotArmParameters{
							.total_arm_length = arm_length,
							.joint_types = joint_type,
							.add_spherical_wrist = false
					});
				}
			}
		}

		return robot_arm_parameters;
	}

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

		mgodpl::Mesh base_mesh = from_dae(ROBOTS_DIR "/meshes/drone.dae");
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
											  .min_angle = -M_PI / 2.0,
											  .max_angle = M_PI / 2.0
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
