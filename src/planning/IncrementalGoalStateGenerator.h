// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef MGODPL_INCREMENTALGOALSTATEGENERATOR_H
#define MGODPL_INCREMENTALGOALSTATEGENERATOR_H

#include <memory>
#include <random_numbers/random_numbers.h>
#include "moveit_forward_declarations.h"
#include "../math/Vec3.h"

namespace mgodpl {

	struct TfTreeNode {
		size_t depth; // How many joints have been determined at this point. (Note the fixed joint at the end!)
		Eigen::Isometry3d link_tf; // FIXME I don't want Eigen stuff in public interfaces.
		std::vector<std::pair<double, size_t>> children;
	};

	struct GoalStateAlgorithm {



		std::shared_ptr<fcl::BVHModel<fcl::OBBd>> tree_bvh;
		std::shared_ptr<fcl::CollisionObjectd> tree_co;
		random_numbers::RandomNumberGenerator _rng;
		moveit::core::RobotModelConstPtr robot;
		math::Vec3d target;

		std::vector<const moveit::core::LinkModel *> kinematic_chain;

		std::vector<TfTreeNode> nodes;

		std::vector<size_t> genRootNode();

		GoalStateAlgorithm(const moveit::core::RobotModelConstPtr &robot,
						   const shape_msgs::msg::Mesh &tree_model,
						   const math::Vec3d &target,
						   const random_numbers::RandomNumberGenerator &rng);

		std::vector<Eigen::Isometry3d> free_link_tfs(
				const std::vector<double> &joint_values,
				const Eigen::Isometry3d &locked_link_tf);

		std::vector<double> revolute_joint_gradients(const std::vector<double> &jointValues,
													 const std::vector<Eigen::Vector3d> &local_axes,
													 const std::vector<Eigen::Isometry3d> &free_link_tfs,
													 Eigen::AngleAxisd &ideal_rotation) const;

		/**
		 * @brief Adjust the given joint values to bring the base upright, such that the local
		 * z-axis of the base link aligns with the global z-axis.
		 *
		 * TODO: Right now, we use a purely numeric method, but an analytic solution should be possible when there are 2 DOFs left.
		 *
		 * @param locked_revjoints		The number of locked revolute joints.
		 * @param locked_link_tf 		The transform of the link furthest from the end-effector that is locked in place.
		 * @param jointValues 			The joint values of the free revolute joints. (To be adjusted in place.)
		 * @return 						True if the base was brought upright, false otherwise.
		 */
		bool bringBaseUpright(const size_t locked_revjoints,
							  const Eigen::Isometry3d &locked_link_tf,
							  std::vector<double> &jointValues
		);

		void
		try_branch_from(size_t current_node);

		void iterate();

	};
};


#endif //MGODPL_INCREMENTALGOALSTATEGENERATOR_H
