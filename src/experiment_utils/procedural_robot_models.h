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
		VERTICAL ///< Represents a vertical joint.
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

		/**
		 * Compile a short string corresponding to the parameters.
		 */
		[[nodiscard]] std::string short_designator() const;
	};

	/**
	 * @struct RobotArmMetaParameters
	 * @brief A structure representing the meta-parameters for generating robot arm parameters.
	 *
	 * This structure contains the lengths of the arms, the maximum number of links,
	 * and flags indicating whether to include all horizontal, all vertical, and alternating horizontal/vertical joints.
	 */
	struct RobotArmMetaParameters {
		std::array<double, 4> arm_lengths; ///< An array of arm lengths.
		size_t max_links; ///< The maximum number of links in the arm.
		bool include_all_horizontal; ///< Flag to include all horizontal joints.
		bool include_all_vertical; ///< Flag to include all vertical joints.
		bool include_alternating_horizontal_vertical; ///< Flag to include alternating horizontal/vertical joints.
	};

	/**
	 * @brief Generates a vector of RobotArmParameters based on the given meta-parameters.
	 *
	 * This function generates a vector of RobotArmParameters by iterating over the arm lengths and the maximum number of links,
	 * and including the specified joint types based on the meta-parameters.
	 *
	 * @param meta_params The meta-parameters for generating robot arm parameters.
	 * @return A vector of RobotArmParameters.
	 */
	std::vector<RobotArmParameters> generateRobotArmParameters(
		const RobotArmMetaParameters &meta_params);

	/**
	 * @brief Creates a procedural robot model based on the given robot arm parameters.
	 *
	 * This function creates a procedural robot model using the specified robot arm parameters.
	 *
	 * @param parameters The parameters of the robot arm.
	 * @return A procedural robot model.
	 */
	robot_model::RobotModel createProceduralRobotModel(const RobotArmParameters &parameters);

	/**
	 * @brief Creates a default procedural robot model.
	 *
	 * This function creates a procedural robot model using default parameters.
	 *
	 * @return A default procedural robot model.
	 */
	robot_model::RobotModel createProceduralRobotModel();
}

#endif //MGODPL_PROCEDURAL_ROBOT_MODELS_H
