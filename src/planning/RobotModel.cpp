// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/23/23.
//

#include "RobotModel.h"

#include <numeric>

#include "RobotState.h"

namespace mgodpl::robot_model {

	ForwardKinematicsResult forwardKinematics(const RobotModel &model,
											  const std::vector<double> &joint_values,
											  const RobotModel::LinkId &root_link,
											  const math::Transformd &root_link_transform) {

		// Allocate a result with all identity transforms.
		ForwardKinematicsResult result{
				.link_transforms = std::vector<math::Transformd>(model.getLinks().size(), root_link_transform)};

		// Keep a vector to track which links have been visited, since a robot may contain cycles.
		std::vector<bool> visited(model.getLinks().size(), false);

		// Keep a stack of links to visit, and push the root link.
		// Every entry in the stack is a pair of a link and its transform relative to the root_link.
		std::vector<std::pair<RobotModel::LinkId, math::Transformd>> stack{{root_link, root_link_transform}};

		// Keep track of the index in the joint_values vector; this induces the DFS order.
		size_t variable_index = 0;

		while (!stack.empty()) {

			// Pop the top of the stack.
			auto [link, transform] = stack.back();
			stack.pop_back();

			// If we have already visited this link, skip it.
			if (visited[link])
				continue;

			// Mark the link as visited.
			visited[link] = true;

			// Update the transform for this link.
			result.link_transforms[link] = transform;

			// For all children, if not already visited, compute their transform and push them on the stack.
			for (const auto &joint: model.getLinks()[link].joints) {

				// If the other link is already visited, skip it.
				if (visited[model.getJoints()[joint].linkB])
					continue;

				// Compute the transform from the link to the joint.
				math::Transformd local_to_joint = model.getJoints()[joint].linkA == link
														  ? model.getJoints()[joint].attachmentA
														  : model.getJoints()[joint].attachmentB;

				// Compute the transform induced by the variable part of the joint.
				math::Transformd joint_transform = RobotModel::variableTransform(model.getJoints()[joint].type_specific,
																				 joint_values.data() + variable_index);
				if (model.getJoints()[joint].linkB == link)
					joint_transform = joint_transform.inverse();
				variable_index += RobotModel::n_variables(model.getJoints()[joint].type_specific);

				// Compute the transform from the joint to the link.
				math::Transformd after_joint_frame =
						(model.getJoints()[joint].linkA == link ? model.getJoints()[joint].attachmentB
																: model.getJoints()[joint].attachmentA)
								.inverse();

				// Compute the transform from the link to the next link.
				math::Transformd next_link_transform =
						transform.then(local_to_joint).then(joint_transform).then(after_joint_frame);

				// Push the next link on the stack.
				stack.emplace_back(model.getJoints()[joint].linkA == link ? model.getJoints()[joint].linkB
																		  : model.getJoints()[joint].linkA,
								   next_link_transform);
			}
		}

		// Return the result.
		return result;
	}
	ForwardKinematicsResult
	forwardKinematics(const RobotModel &model, const RobotState &state, const RobotModel::LinkId &root_link) {
		return forwardKinematics(model, state.joint_values, root_link, state.base_tf);
	}

	RobotModel::LinkId RobotModel::insertLink(const RobotModel::Link &link) {
		// Just push the link onto the vector, and return the index.
		links.push_back(link);

		return links.size() - 1;
	}

	RobotModel::JointId RobotModel::insertJoint(const RobotModel::Joint &joint) {
		// Just push the joint onto the vector, and return the index.
		joints.push_back(joint);

		// Create the backlinks.
		links[joint.linkA].joints.push_back(joints.size() - 1);
		links[joint.linkB].joints.push_back(joints.size() - 1);

		return joints.size() - 1;
	}

	RobotModel::LinkId RobotModel::findLinkByName(const std::string &name) const {
		// Linear search; throw an exception if not found.
		for (size_t i = 0; i < links.size(); i++)
			if (links[i].name == name)
				return i;
		throw std::runtime_error("Link not found: " + name);
	}

	RobotModel::JointId RobotModel::findJointByName(const std::string &name) const {
		// Linear search; throw an exception if not found.
		for (size_t i = 0; i < joints.size(); i++)
			if (joints[i].name == name)
				return i;
		throw std::runtime_error("Joint not found: " + name);
	}

	size_t RobotModel::count_joint_variables() const {
		return std::accumulate(joints.begin(), joints.end(), 0, [](size_t acc, const auto &joint) {
			return acc + n_variables(joint.type_specific);
		});
	}

	mgodpl::math::Transformd
	mgodpl::robot_model::RobotModel::variableTransform(const RobotModel::JointTypeSpecific &variablePart,
													   const double *joint_values) {
		// Switch on the type of joint.
		switch (variablePart.index()) {
			case 0: {
				// Construct a rotation transform from the axis and the joint value.
				const auto &revolute_joint = std::get<RevoluteJoint>(variablePart);
				return math::Transformd::fromRotation(
						math::Quaterniond::fromAxisAngle(revolute_joint.axis, joint_values[0]));
			}
			case 1: {
				// Fixed joint, return identity.
				return math::Transformd::identity();
			}
			default:
				throw std::runtime_error("Unknown joint type");
		}
	}

	RobotModel::Link RobotModel::Link::namedBox(const std::string &name, const math::Vec3d &size) {
		return Link{.name = name,
					.joints = {},
					.collision_geometry = {PositionedShape::untransformed(Box{.size = size})},
					.visual_geometry = {}};
	}
} // namespace mgodpl::robot_model
