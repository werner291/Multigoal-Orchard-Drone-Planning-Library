// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/20/23.
//


/**
 * A reduced, no-indirection (except for the links vector) version of the moveit::core::RobotModel class,
 * just enough to describe the kinematic chain from the base link to the end-effector, not including
 * any weirdness from fixed joints or the floating joint.
 *
 * Assumes all joints are revolute, with a unit axis that is either +X, +Y or +Z.
 *
 * Note: the idea is somewhat similar to the DH-parameterization,
 * but the robot is simple enough that we can reduce further.
 */
struct SimpleKinematicChain {

	enum XYZ {
		X,
		Y,
		Z
	};

	struct Link {
		XYZ axis;
		double length;
	};

	std::vector<Link> links;

};

/**
 * A simple vector of joint angles for the SimpleKinematicChain.
 */
struct SimpleJointAngles {
	std::vector<double> angles;
};

/**
 * Extract a SimpleKinematicChain from a moveit::core::RobotModel, getting rid of all the extra bits.
 *
 * @param rm 		The robot model.
 * @return 			The kinematic chain.
 */
SimpleKinematicChain fromRobotModel(const moveit::core::RobotModel &rm) {

	// Get the end-effector.
	const moveit::core::LinkModel *lm = rm.getLinkModel("end_effector");

	// Allocate an empty chain.
	SimpleKinematicChain chain;

	// A variable to accumulate length as we travel up the chain; every time we pass a joint,
	// we reset this to 0 and add a new link to the chain.
	double jointLength = 0.0;

	// Walk up the chain, until we reach the base link. We ignore the base link's floating joint.
	while (lm->getParentLinkModel() != nullptr) {

		// Get the parent joint; there always is one.
		const auto &joint = lm->getParentJointModel();

		// Distinguish betweem joint types; only fixed and revolute are supported.
		if (joint->getType() == moveit::core::JointModel::REVOLUTE) {

			const auto &axis = dynamic_cast<const moveit::core::RevoluteJointModel *>(joint)->getAxis();

			SimpleKinematicChain::XYZ axis_id =
					axis.x() == 1 ? SimpleKinematicChain::X :
					axis.y() == 1 ? SimpleKinematicChain::Y :
					axis.z() == 1 ? SimpleKinematicChain::Z :
					throw std::runtime_error("Invalid axis; assumed positive unit axis.");

			chain.links.push_back({
										  axis_id,
										  jointLength
								  });

			jointLength = 0.0;

		} else if (joint->getType() != moveit::core::JointModel::FIXED) {
			throw std::runtime_error("Unsupported joint type.");
		}

		jointLength += lm->getJointOriginTransform().translation().norm();

		lm = lm->getParentLinkModel();
	}

	return chain;

}