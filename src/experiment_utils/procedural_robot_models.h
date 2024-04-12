// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef MGODPL_PROCEDURAL_ROBOT_MODELS_H
#define MGODPL_PROCEDURAL_ROBOT_MODELS_H

#include "../planning/RobotModel.h"

namespace mgodpl::experiments {

	// The length of the arm in the robot model.
	static const double ARM_LENGTH = 0.75;

/**
	 * @enum JointType
	 * @brief An enumeration representing whether a joint is horizontal or vertical.
	 */
	enum JointType {
		HORIZONTAL, ///< Represents a horizontal joint.
		VERTICAL    ///< Represents a vertical joint.
	};

	/**
	 * @struct RobotArmParameters
	 * @brief A structure representing the parameters of a robot arm.
	 *
	 * This structure contains the total length of the arm, the types of the joints in the arm,
	 * and a flag indicating whether to add a spherical wrist to the arm.
	 */
	struct RobotArmParameters {
		double total_arm_length; ///< The total length of the arm.
		std::vector<JointType> joint_types; ///< The types of the joints in the arm.
		bool add_spherical_wrist; ///< A flag indicating whether to add a spherical wrist to the arm.

		/**
		 * @brief Calculates and returns the length of a segment of the arm.
		 *
		 * This function calculates the length of a segment of the arm by dividing the total length of the arm
		 * by the number of joints in the arm.
		 *
		 * @return The length of a segment of the arm.
		 */
		[[nodiscard]] double arm_segment_length() const {
			return total_arm_length / (double) joint_types.size();
		}
	};

	mgodpl::robot_model::RobotModel createProceduralRobotModel(const RobotArmParameters &parameters);

	mgodpl::robot_model::RobotModel createProceduralRobotModel();
}

#endif //MGODPL_PROCEDURAL_ROBOT_MODELS_H
