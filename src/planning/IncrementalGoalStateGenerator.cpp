// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/22/23.
//

#include <Eigen/Geometry>
#include <moveit/robot_model/link_model.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <shape_msgs/msg/mesh.hpp>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/geometry/shape/box.h>
#include <geometric_shapes/shapes.h>
#include "IncrementalGoalStateGenerator.h"
#include "JointSpacePoint.h"
#include "moveit_state_tools.h"
#include "../experiment_utils/fcl_utils.h"


bool
checkLinkCollision(fcl::CollisionObjectd &tco, const moveit::core::LinkModel *lm, const Eigen::Isometry3d &transform) {

	std::shared_ptr<const shapes::Box> box_shape = std::dynamic_pointer_cast<const shapes::Box>(lm->getShapes()[0]);

	assert(box_shape != nullptr);

	auto fcl_box = std::make_shared<fcl::Boxd>(box_shape->size[0], box_shape->size[1], box_shape->size[2]);

	// Color the link red or green depending on trunk collisions.
	fcl::Vector3d contact_point;
	fcl::CollisionObjectd co(fcl_box, transform);
	fcl::CollisionRequestd req;
	fcl::CollisionResultd res;
	fcl::collide(&tco, &co, req, res);

	return res.isCollision();
}


std::vector<const moveit::core::LinkModel *> mkInvertedKinematicChain(const moveit::core::RobotModelConstPtr &robot) {
	std::vector<const moveit::core::LinkModel *> kinematic_chain{
			robot->getLinkModel("end_effector")
	};

	while (kinematic_chain.back()->getParentLinkModel() != nullptr) {
		kinematic_chain.push_back(kinematic_chain.back()->getParentLinkModel());
	}

	return kinematic_chain;
}
/**
 * Computes a minimal rotation to get the given orientation to be upright.
 *
 * @param orientation The orientation to get upright.
 * @return The rotation to apply to get the given orientation to be upright.
 */
Eigen::AngleAxisd rotation_to_upright(const Eigen::Isometry3d::RotationReturnType &orientation) {
	Eigen::Vector3d local_up = orientation * Eigen::Vector3d(0.0, 0.0, 1.0);
	Eigen::Vector3d ideal_axis = local_up.cross(Eigen::Vector3d(0.0, 0.0, 1.0)).normalized();

	double ideal_angle = std::acos(std::clamp(local_up.dot(Eigen::Vector3d(0.0, 0.0, 1.0)), -1.0, 1.0));

	Eigen::AngleAxisd ideal_rotation(ideal_angle, ideal_axis);

	// Sanity check: the rotation should bring the local up to UNIT_Z.
	assert(std::abs((ideal_rotation * local_up).z() - 1.0) < 0.0001);

	return ideal_rotation;
}

/**
 * @brief Given a child_link and the global transform thereof, compute the transform of the parent joint.
 *
 * @param joint_value 			The value of the joint.
 * @param child_link 			The child_link to compute the parent joint transform for.
 * @param child_tf 				The global transform of the child_link.
 * @return 						The transform of the parent joint.
 */
Eigen::Isometry3d parent_tf_from_child(
		const double joint_value,
		const moveit::core::LinkModel &child_link,
		const Eigen::Isometry3d &child_tf) {

	// Get the joint between the child_link and its parent.
	auto parent_joint_model = child_link.getParentJointModel();

	// Make sure it's a <= 1 DOF joint; otherwise we'll read past the joint_value.
	assert(parent_joint_model->getVariableCount() <= 1);

	// Compute the transform induced by the joint with the given joint value.
	Eigen::Isometry3d joint_tf;
	parent_joint_model->computeTransform(&joint_value, joint_tf);

	return child_tf * joint_tf.inverse() * child_link.getJointOriginTransform().inverse();
}

namespace mgodpl {


	std::vector<size_t> GoalStateAlgorithm::genRootNode() {

		moveit_facade::JointSpacePoint gs = experiment_state_tools::genGoalSampleUniform(target, _rng, *robot);

		std::vector<double> joint_values;
		joint_values.insert(joint_values.end(),
							gs.joint_values.begin() + 7 /*Skip the base link.*/,
							gs.joint_values.end());
		std::reverse(joint_values.begin(), joint_values.end());

		moveit::core::RobotState gs_mt(robot);
		gs.to_moveit(gs_mt);

		Eigen::Isometry3d end_effector_tf = gs_mt.getGlobalLinkTransform("end_effector");

		std::vector<size_t> new_nodes;

		// We'll work out way down the chain from the end-effector to the base.
		for (size_t depth = 1; depth < kinematic_chain.size(); ++depth) {

			const moveit::core::LinkModel *child_link = kinematic_chain[depth - 1];

			// We use 0 for the fixed joint; otherwise, we use one of the revolute joint values.
			// Note: the joint value is the value of the joint that connect the link `link` to its child.
			// At depth == 1, that would be the value of the fixed joint between the end-effector and the last link.
			double joint_value = depth == 1 ? 0.0 : joint_values[depth - 2];

			// Compute the transform of the link, given the joint value and the transform of the child link.
			Eigen::Isometry3d link_tf = parent_tf_from_child(joint_value, *child_link, depth == 1 ? end_effector_tf
																								  : nodes.back().link_tf);

			if (checkLinkCollision(*tree_co, child_link->getParentLinkModel(), link_tf)) {
				std::cout << "Collision after " << depth << " joints!" << std::endl;
				break;
			}

			if (depth > 1) {
				nodes.back().children.emplace_back(joint_value, nodes.size());
			}

			nodes.push_back(TfTreeNode{depth, link_tf, {}});
			new_nodes.push_back(nodes.size() - 1);

		}

		return new_nodes;
	}

	GoalStateAlgorithm::GoalStateAlgorithm(const moveit::core::RobotModelConstPtr &robot,
										   const shape_msgs::msg::Mesh &tree_model,
										   const math::Vec3d &target,
										   const random_numbers::RandomNumberGenerator &rng) :
			tree_bvh(fcl_utils::meshToFclBVH(tree_model)),
			tree_co(std::make_shared<fcl::CollisionObjectd>(tree_bvh, fcl::Transform3d::Identity())),
			_rng(rng),
			robot(robot),
			target(target),
			kinematic_chain(mkInvertedKinematicChain(robot)) {

	}

	std::vector<double> GoalStateAlgorithm::revolute_joint_gradients(const std::vector<double> &jointValues,
																	 const std::vector<Eigen::Vector3d> &local_axes,
																	 const std::vector<Eigen::Isometry3d> &free_link_tfs,
																	 Eigen::AngleAxisd &ideal_rotation) const {
		std::vector<double> gradients;
		gradients.reserve(jointValues.size());

		// For each joint, compute the alignment with the ideal rotation angle.
		for (size_t i = 0; i < jointValues.size(); ++i) {
			// Compute the axis of the revolute joint in the global frame of reference.
			Eigen::Vector3d actual_axis =
					free_link_tfs[i].rotation()  // <- The rotation of the parent link.
					*
					local_axes[i];             // <- The axis of the revolute joint in the parent link's frame of reference.

			// The gradient factor, or how strongly a rotation in this revolution joint contributes to the ideal rotation.
			double gradient = ideal_rotation.axis().dot(actual_axis);

			gradients.push_back(gradient);
		}

		return gradients;
	}

	bool GoalStateAlgorithm::bringBaseUpright(const size_t locked_revjoints,
											  const Eigen::Isometry3d &locked_link_tf,
											  std::vector<double> &jointValues) {

		// The robot has 3 revolute joints. As a sanity check, make sure that the number of locked joints plus the number of free joints is 3.
		// Note: if there are 3 free links, then the base joint will move!
		assert(locked_revjoints + jointValues.size() == 3);
		assert(kinematic_chain[1]->getParentJointModel()->getType() ==
			   moveit::core::JointModel::REVOLUTE); // <- Note, this link is always fixed in this context.
		assert(kinematic_chain[2]->getParentJointModel()->getType() == moveit::core::JointModel::REVOLUTE);
		assert(kinematic_chain[3]->getParentJointModel()->getType() == moveit::core::JointModel::REVOLUTE);
		assert(kinematic_chain[4]->getParentJointModel()->getType() ==
			   moveit::core::JointModel::FLOATING); // <- This is the base joint.

		// Step 1: find the axes of the free revolute joints. (We can probably cache this, or at least avoid allocating a new vector every time?)
		// Note: these are in the frame of reference of the parent link! (And the child link due to the nature of revolute joints.)
		std::vector<Eigen::Vector3d> local_axes;
		for (size_t i = 0; i < jointValues.size(); ++i) {
			assert(kinematic_chain[i + 1]->getParentJointModel()->getType() == moveit::core::JointModel::REVOLUTE);
			Eigen::Vector3d axis = dynamic_cast<const moveit::core::RevoluteJointModel *>(kinematic_chain[i +
																										  1]->getParentJointModel())->getAxis();
			local_axes.push_back(axis);
		}

		// Step 2: compute the transforms of the free links.
		std::vector<Eigen::Isometry3d> free_link_tfs = this->free_link_tfs(jointValues, locked_link_tf);

		Eigen::AngleAxisd ideal_rotation = rotation_to_upright(free_link_tfs.back().rotation());

		double error = std::abs(ideal_rotation.angle());
		double error_change = 0.0;

		do {

			// Compute the influence of every free joint on the rotation of the base.
			std::vector<double> gradients = revolute_joint_gradients(jointValues,
																	 local_axes,
																	 free_link_tfs,
																	 ideal_rotation);

			// Avoid taking steps that are too large.
			double step_size = std::min(0.1, error);

			// Step 3: adjust the joint values by applying the gradients.
			for (size_t i = 0; i < jointValues.size(); ++i) {
				jointValues[i] = std::clamp(jointValues[i] - gradients[i] * step_size, -M_PI / 2.0, M_PI / 2.0);
			}

			// Recompute the transforms of the free links.
			free_link_tfs = this->free_link_tfs(jointValues, locked_link_tf);

			// Recompute the ideal rotation.
			ideal_rotation = rotation_to_upright(free_link_tfs.back().rotation());

			// Recompute the error.
			double remaining_error = std::abs(ideal_rotation.angle());
			error_change = error - remaining_error;
			error = remaining_error;

		} while (error > 0.01 && error_change > 0.001);
		// Continue until either the error is small enough, or we're not making enough progress anymore.

		return error < 0.01;
	}

	void GoalStateAlgorithm::try_branch_from(size_t current_node) {// Find out how many free joints remain, assuming that the link represented by the current node is locked in place.
// The calculation works as follows:
// 		If current_node->depth == 1, then we only passed the fixed joint, so there are 3 free revolute joints.
//			Thus, kinematic_chain.size() - 1 == 3, since we need to ignore the base joint (which uses a floating parent joint).
		int remaining_joints = kinematic_chain.size() - 1 - nodes[current_node].depth;

		if (remaining_joints == 0) {
			return;
		}

		// Generate random joint values for the remaining joints.
		std::vector<double> joint_values(remaining_joints);
		for (double &joint_value: joint_values) {
			joint_value = _rng.uniformReal(-M_PI / 2.0, M_PI / 2.0);
		}

		// Push the base upright; this may fail if the IK problem either has no solution, or the solver gets stuck in a local minimum.
// Though, since we pick joint values at random, we should eventually find a solution on a later attempt.
		bool can_solve_ik = bringBaseUpright(nodes[current_node].depth - 1,
											 nodes[current_node].link_tf,
											 joint_values);

		// If we can solve the IK problem, add a new chain of nodes to the tree.
		if (can_solve_ik || true) {

			// Create a new subtree with the given joint values.
			for (double &joint_value: joint_values) {

				// Compute the transform of the new node.
				const moveit::core::LinkModel *&child_link = kinematic_chain[nodes[current_node].depth];

				// Make sure we're actually using a revolute joint, since there's so much indexing magic.
				assert(child_link->getParentJointModel()->getType() == moveit::core::JointModel::REVOLUTE);

				Eigen::Isometry3d tf = parent_tf_from_child(joint_value, *child_link, nodes[current_node].link_tf);

				bool collides = checkLinkCollision(*tree_co, child_link->getParentLinkModel(), tf);

				if (collides) {
					std::cout << "Collision!" << std::endl;
					break;
				}

				// Add the new node.
				nodes[current_node].children.emplace_back(joint_value, nodes.size());
				nodes.push_back(TfTreeNode{nodes[current_node].depth + 1, tf, {}});

				// Move to the new node.
				current_node = nodes.size() - 1;
			}
		}
	}

	void GoalStateAlgorithm::iterate() {
		const auto &new_nodes = genRootNode();

		if (!new_nodes.empty()) {
			try_branch_from(new_nodes[0]);
		}
	}

	std::vector<Eigen::Isometry3d>
	GoalStateAlgorithm::free_link_tfs(const std::vector<double> &joint_values, const Eigen::Isometry3d &locked_link_tf) {

		std::vector<Eigen::Isometry3d> free_link_tfs;

		for (size_t i = 0; i < joint_values.size(); ++i) {
			// Get the child link, the parent of which we'd like to compute the transform for.
			// The logic behind the indexing here: if i == 0 and locked_revjoints == 0,
			// then i + locked_revjoints + 1 == 1, which is the index of the first link with a revolute parent joint.

			const auto &child_link = kinematic_chain[i + 1];
			const auto &child_tf = i == 0 ? locked_link_tf : free_link_tfs.back();

			assert(child_link->getParentJointModel()->getType() == moveit::core::JointModel::REVOLUTE);

			free_link_tfs.push_back(parent_tf_from_child(joint_values[i],
														 *child_link,
														 child_tf));
		}

		return free_link_tfs;
	}
}
