// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/23/23.
//

#ifndef MGODPL_ROBOTMODEL_H
#define MGODPL_ROBOTMODEL_H

#include <variant>
#include <vector>
#include <string>

#include "../math/Transform.h"
#include "../experiment_utils/shapes.h"
#include "../experiment_utils/positioned_shape.h"

/**
 * @brief This namespace contains all code related to modeling robots.
 *
 * This provides an alternative implementation to moveit's robot model.
 */
namespace mgodpl::robot_model {

	/**
	 * @brief A robot model, consisting of links and joints.
	 *
	 * Note: there is no absolute frame of reference; the model is defined in terms of relative transformations between links and joints.
	 */
	class RobotModel {

	public:
		/// The index of a link in the links vector. Will remain valid as long as links are not removed.
		using LinkId = size_t;

		/// The index of a joint in the joints vector. Will remain valid as long as joints are not removed.
		using JointId = size_t;

		/**
		 * @brief A revolute joint, which can rotate around a single axis. The joint has a minimum and maximum angle.
		 */
		struct RevoluteJoint {
			math::Vec3d axis;
			double min_angle;
			double max_angle;
		};

		/**
		 * @brief A fixed joint; this is effectively a joint with no degrees of freedom; links are welded together.
		 */
		struct FixedJoint {

		};

		/**
		 * @brief A joint, which connects two links.
		 */
		using JointTypeSpecific = std::variant<RevoluteJoint, FixedJoint>;

		/**
		 * @brief A joint that connects two specific links.
		 */
		struct Joint {

			/// The name of the joint.
			std::string name;

			/// The transformations from the link's origin to the joint's frame of reference. (Note that you may have to inverse these in some cases.)
			math::Transformd attachmentA, attachmentB;

			/// The IDs of the links that this joint connects.
			LinkId linkA, linkB;

			/// The type-specific data of the joint, such as the axis of rotation for a revolute joint.
			JointTypeSpecific type_specific;

		};

		/**
		 * @brief Returns the transform induced by the joint in the joint's local frame of reference.
		 *
		 * @param variablePart	The type-specific data of the joint.
		 * @param joint_values 	The values of the joint variables; make sure there are enough values (consult n_variables()).
		 * @return 				The transform induced by the joint in the joint's local frame of reference.
		 */
		static math::Transformd variableTransform(const JointTypeSpecific& variablePart, const double* joint_values);

		/**
		 * @brief Returns the number of variables that this joint has.
		 * @return The number of variables that this joint has.
		 */
		[[nodiscard]] static inline size_t n_variables(const JointTypeSpecific& variablePart) {
			switch (variablePart.index()) {
				case 0: return 1; // Revolute joint
				case 1: return 0; // Fixed joint
				default: throw std::runtime_error("Unknown joint type");
			}
		}

		/**
		 * @brief A link, which is a rigid body that can be connected to other links via joints.
		 */
		struct Link {

			/// The name of the link.
			std::string name;

			/// The joints that connect this link to other links (IDs into the joints vector).
			std::vector<JointId> joints {};

			/// The collision geometry of the link.
			std::vector<PositionedShape> collision_geometry {};

			/// The visual geometry of the link.
			std::vector<PositionedShape> visual_geometry {};
		};

		/**
		 * @brief Inserts a link into the model and returns its ID.
		 * @param link 		The link to insert.
		 * @return 			The ID of the inserted link.
		 */
		LinkId insertLink(const Link &link);

		/**
		 * @brief Inserts a joint into the model and returns its ID.
		 * @param joint 	The joint to insert.
		 * @return 			The ID of the inserted joint.
		 */
		JointId insertJoint(const Joint &joint);

		/**
		 * @brief Finds a link by name and returns its ID. (Linear search; you might want to index links by name if you use this a lot.)
		 * @param name 		The name of the link to find.
		 *
		 * @throws std::runtime_error if the link is not found.
		 *
		 * @return 			The ID of the link.
		 */
		[[nodiscard]] LinkId findLinkByName(const std::string &name) const;

		/**
		 * @brief Finds a joint by name and returns its ID. (Linear search; you might want to index joints by name if you use this a lot.)
		 * @param name 		The name of the joint to find.
		 *
		 * @throws std::runtime_error if the joint is not found.
		 *
		 * @return 			The ID of the joint.
		 */
		[[nodiscard]] JointId findJointByName(const std::string &name) const;

		[[nodiscard]] size_t count_joint_variables() const;

		/**
		 * @brief	Returns the links in the model.
		 *
		 * @return  The links in the model as a read-only vector.
		 */
		[[nodiscard]] const std::vector<Link>& getLinks() const {
			return links;
		}

		/**
		 * @brief	Returns the joints in the model.
		 * @return	The joints in the model as a read-only vector.
		 */
		[[nodiscard]] const std::vector<Joint>& getJoints() const {
			return joints;
		}

	private:

		/// The links in the model.
		std::vector<Link> links;

		/// The joints in the model.
		std::vector<Joint> joints;


	};

	/**
	 * @brief A result of a forward kinematics computation.
	 */
	struct ForwardKinematicsResult {
		std::vector<math::Transformd> link_transforms;

		/**
		 * @brief Returns the transform of a link.
		 *
		 * @param link 		The ID of the link, corresponding to the index in the links vector of the RobotModel.
		 * @return 			The transform of the link.
		 */
		[[nodiscard]] const math::Transformd& forLink(RobotModel::LinkId link) const {
			return link_transforms[link];
		}
	};

	/**
	 * @brief Computes the forward kinematics of a robot model, given a set of joint values and a link from which to compute the forward kinematics.
	 *
	 * @param model 				The robot model.
	 * @param joint_values 			The joint values (in DFS order from the root_link).
	 * @param root_link 			The link from which to compute the forward kinematics.
	 * @param from_link_transform	The transform of the root_link relative to the root link (if identity, the computed transforms will be relative to the root_link).
	 * @return 						The forward kinematics result.
	 */
	ForwardKinematicsResult forwardKinematics(const RobotModel &model,
											  const std::vector<double> &joint_values,
											  const RobotModel::LinkId& root_link,
											  const math::Transformd& from_link_transform = math::Transformd::identity());

}

#endif //MGODPL_ROBOTMODEL_H
