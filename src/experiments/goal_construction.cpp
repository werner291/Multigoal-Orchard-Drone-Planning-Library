//
// Created by werner on 17-11-23.
//

#include <vtkProperty.h>
#include <moveit/robot_model/link_model.h>
#include <moveit/robot_model/robot_model.h>
#include <random_numbers/random_numbers.h>
#include <moveit/robot_state/robot_state.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/narrowphase/collision-inl.h>
#include <fcl/narrowphase/collision_request.h>
#include <geometric_shapes/shapes.h>
#include <vtkRenderer.h>

#include "../experiment_utils/load_robot_model.h"
#include "../experiment_utils/mesh_utils.h"
#include "../planning/moveit_state_tools.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/VtkRobotModel.h"
#include "../visualization/quick_markers.h"
#include "../planning/JointSpacePoint.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/fcl_utils.h"

using namespace mgodpl;

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

Eigen::Isometry3d do_reversed_fk(
		const double joint_value,
		const moveit::core::LinkModel &link,
		const Eigen::Isometry3d &end_effector_tf) {


	auto parent_joint_model = link.getParentJointModel();

	auto joint = parent_joint_model->getType();

	switch (joint) {
		case moveit::core::JointModel::FIXED: {
			return end_effector_tf * link.getJointOriginTransform().inverse();
		}
			break;

		case moveit::core::JointModel::REVOLUTE: {

			Eigen::Isometry3d joint_tf;
			parent_joint_model->computeTransform(&joint_value, joint_tf);

			return end_effector_tf * joint_tf.inverse() * link.getJointOriginTransform().inverse();
		}
			break;

		default:
			throw std::runtime_error("Unsupported joint type.");

	}
}

/**
 * Applies the given Eigen transform to the specified vtkActor.
 *
 * @param transform The Eigen transform to apply.
 * @param actor The vtkActor to apply the transform to.
 */
void applyEigenTransformToActor(Eigen::Isometry3d &transform, vtkActor *actor) {
	actor->SetPosition(
			transform.translation().x(),
			transform.translation().y(),
			transform.translation().z());

	Eigen::AngleAxisd tfRot(transform.rotation());

	actor->SetOrientation(0.0, 0.0, 0.0);

	actor->RotateWXYZ(tfRot.angle() / M_PI * 180.0,
					  tfRot.axis().x(),
					  tfRot.axis().y(),
					  tfRot.axis().z());
}

/**
 * @brief Compute the rough center of a robot based on the given link transformations.
 *
 * This function computes the rough center of a robot based on the provided link transformations.
 * The rough center is calculated by taking the average position of all link transformations.
 *
 * @param link_tfs A vector containing the link transformations of the robot.
 *                 The link transformations should be of type Eigen::Isometry3d.
 *
 * @return The computed rough center of the robot as an math::Vec3d.
 */
math::Vec3d computeRoughRobotCenter(std::vector<Eigen::Isometry3d> &link_tfs) {
	math::Vec3d mean_center{0.0, 0.0, 0.0};
	for (const auto &link_tf: link_tfs) {
		mean_center = mean_center +
					  math::Vec3d(link_tf.translation().x(), link_tf.translation().y(), link_tf.translation().z());
	}
	mean_center = mean_center / (double) link_tfs.size();
	return mean_center;
}

std::vector<vtkSmartPointer<vtkActor>>
mkLinkActors(std::vector<const moveit::core::LinkModel *> &kinematic_chain, SimpleVtkViewer &viewer) {
	std::vector<vtkSmartPointer<vtkActor>> link_actors;

	for (auto &i1: kinematic_chain) {
		link_actors.push_back(visualization::VtkRobotModel::actorForLink({0.8, 0.8, 0.8}, i1));
		viewer.addActor(link_actors.back());
	}
	return link_actors;
}

void applyActorTransforms(
		const std::vector<const moveit::core::LinkModel *> &kinematic_chain,
		const std::vector<Eigen::Isometry3d> &link_tfs,
		const std::vector<vtkSmartPointer<vtkActor>> &link_actors) {
	for (size_t i1 = 1; i1 < kinematic_chain.size(); ++i1) {
		Eigen::Isometry3d transform = link_tfs[i1] * kinematic_chain[i1]->getCollisionOriginTransforms()[0];
		applyEigenTransformToActor(transform, link_actors[i1]);
	}
}

void checkLinkCollision(fcl::CollisionObjectd &tco,
						const std::vector<const moveit::core::LinkModel *> &kinematic_chain,
						const std::vector<vtkSmartPointer<vtkActor>> &link_actors,
						size_t i1,
						const Eigen::Isometry3d &transform) {
	std::shared_ptr<const shapes::Box> box_shape = std::dynamic_pointer_cast<const shapes::Box>(kinematic_chain[i1]->getShapes()[0]);
	assert(box_shape != nullptr);

	auto fcl_box = std::make_shared<fcl::Boxd>(box_shape->size[0], box_shape->size[1], box_shape->size[2]);

	// Color the link red or green depending on trunk collisions.
	fcl::Vector3d contact_point;
	fcl::CollisionObjectd co(fcl_box, transform);
	fcl::CollisionRequestd req;
	fcl::CollisionResultd res;
	fcl::collide(&tco, &co, req, res);

	if (res.isCollision()) {
		link_actors[i1]->GetProperty()->SetColor(1.0, 0.0, 0.0);
	} else {
		link_actors[i1]->GetProperty()->SetColor(0.0, 1.0, 0.0);
	}
}

Eigen::VectorXd computeAlignmentGradient(const std::vector<const moveit::core::LinkModel *> &kinematic_chain,
										 const std::vector<double> &joint_values,
										 const std::vector<Eigen::Isometry3d> &link_tfs,
										 Eigen::AngleAxisd &ideal_rotation,
										 const size_t LOCKED_ANGLES) {

	Eigen::VectorXd adjustment = Eigen::VectorXd::Zero(joint_values.size() - LOCKED_ANGLES);

	for (size_t i = LOCKED_ANGLES; i < joint_values.size(); ++i) {
		Eigen::Vector3d axis = dynamic_cast<const moveit::core::RevoluteJointModel *>(kinematic_chain[i + 1]->getParentJointModel())->getAxis();
		Eigen::Vector3d actual_axis = link_tfs[i + 1].rotation() * axis;
		double alignment = ideal_rotation.axis().dot(actual_axis);

		double joint_adjustment = alignment * ideal_rotation.angle();

		adjustment[i - LOCKED_ANGLES] = joint_adjustment;
	}

	return adjustment;
}

const math::Vec3d WOOD_COLOR{0.5, 0.3, 0.1};
const math::Vec3d FLOOR_COLOR{0.3, 0.6, 0.3};

std::vector<const moveit::core::LinkModel *> mkInvertedKinematicChain(const moveit::core::RobotModelConstPtr &robot) {
	std::vector<const moveit::core::LinkModel *> kinematic_chain{
			robot->getLinkModel("end_effector")
	};

	while (kinematic_chain.back()->getParentLinkModel() != nullptr) {
		kinematic_chain.push_back(kinematic_chain.back()->getParentLinkModel());
	}

	return kinematic_chain;
}

struct GoalStateAlgorithm {

	std::shared_ptr<fcl::BVHModel<fcl::OBBd>> tree_bvh;
	fcl::CollisionObjectd tree_co;
	random_numbers::RandomNumberGenerator _rng;
	const moveit::core::RobotModelConstPtr &robot;
	math::Vec3d target;

	std::vector<const moveit::core::LinkModel *> kinematic_chain;

	struct TfTreeNode {
		size_t depth;
		Eigen::Isometry3d tf;
		std::vector<std::pair<double,TfTreeNode>> children;
	};

	TfTreeNode tf_tree;

	GoalStateAlgorithm(const moveit::core::RobotModelConstPtr &robot,
					   const shape_msgs::msg::Mesh &tree_model,
					   const math::Vec3d &target,
					   const random_numbers::RandomNumberGenerator &rng) :
					   tree_bvh(fcl_utils::meshToFclBVH(tree_model)),
					   tree_co(tree_bvh, fcl::Transform3d::Identity()),
					   _rng(rng),
					   robot(robot),
					   target(target),
					   kinematic_chain(mkInvertedKinematicChain(robot))
					   {

						   moveit_facade::JointSpacePoint gs = experiment_state_tools::genGoalSampleUniform(target, _rng, *robot);

						   std::vector<double> joint_values;
						   joint_values.insert(joint_values.end(), gs.joint_values.begin() + 7 /*Skip the base link.*/, gs.joint_values.end());
						   std::reverse(joint_values.begin(), joint_values.end());

						   // Set to all 0.
//						   for (size_t i = 0; i < joint_values.size(); ++i) {
//							   joint_values[i] = 0.0;
//						   }
//
//						   joint_values[1] = M_PI / 2.0;

						   moveit::core::RobotState gs_mt(robot);
						   gs.to_moveit(gs_mt);

						   Eigen::Isometry3d tf = gs_mt.getGlobalLinkTransform("end_effector");

						   tf_tree = TfTreeNode{0, tf, {}};

						   TfTreeNode *current_node = &tf_tree;

						   for (size_t i = 1; i < kinematic_chain.size(); ++i) {
							   current_node->children.emplace_back(joint_values[i-1], TfTreeNode{i, do_reversed_fk(
									   i == 1 ? 0.0 : joint_values[i-2],
									   *kinematic_chain[i-1],
									   current_node->tf
									   ), {}});
							   current_node = &current_node->children.back().second;
						   }

						   // Check to make sure the last node is upright.

//						   assert(
//								   std::abs(
//										   rotation_to_upright(current_node->tf.rotation()).angle()
//										   ) < 0.01
//								   );


					   }

	bool bringBaseUpright(const Eigen::Isometry3d& locked_link_tf, const size_t LOCKED_ANGLES, std::vector<double> &jointValues) {

		std::vector<Eigen::Isometry3d> free_link_tfs;

		for (size_t i = LOCKED_ANGLES; i < jointValues.size(); ++i) {
			free_link_tfs.push_back(do_reversed_fk(jointValues[i], *kinematic_chain[i], i == LOCKED_ANGLES ? locked_link_tf : free_link_tfs.back()));
		}

		Eigen::AngleAxisd ideal_rotation = rotation_to_upright(free_link_tfs.back().rotation());

		double error = std::abs(ideal_rotation.angle());
		double error_change = 0.0;

		do {

			std::vector<double> gradients;
			gradients.reserve(jointValues.size() - LOCKED_ANGLES);

			// For each joint, compute the alignment with the ideal rotation angle.
			for (size_t i = LOCKED_ANGLES; i < jointValues.size(); ++i) {
				Eigen::Vector3d axis = dynamic_cast<const moveit::core::RevoluteJointModel *>(kinematic_chain[i + 1]->getParentJointModel())->getAxis();
				Eigen::Vector3d actual_axis = free_link_tfs[i + 1].rotation() * axis;
				gradients.push_back(ideal_rotation.axis().dot(actual_axis));
			}

			for (size_t i = LOCKED_ANGLES; i < jointValues.size(); ++i) {
				jointValues[i] = std::clamp(jointValues[i] - gradients[i - LOCKED_ANGLES] * 0.1, -M_PI / 2.0, M_PI / 2.0);
			}

			free_link_tfs.clear();

			for (size_t i = LOCKED_ANGLES; i < jointValues.size(); ++i) {
				free_link_tfs.push_back(do_reversed_fk(jointValues[i], *kinematic_chain[i], i == LOCKED_ANGLES ? locked_link_tf : free_link_tfs.back()));
			}

			ideal_rotation = rotation_to_upright(free_link_tfs.back().rotation());

			double remaining_error = std::abs(ideal_rotation.angle());
			error_change = error - remaining_error;
			error = remaining_error;
		} while (error > 0.01 && error_change > 0.001);

		return error < 0.01;
	}

	void iterate() {

		//Let's create a branch in the tree.
		const TfTreeNode *current_node = &tf_tree;

		// Make a random variant of one of the children.
		double jv = _rng.uniformReal(-M_PI / 2.0, M_PI / 2.0);

		// Compute that child's transform.
		Eigen::Isometry3d tf = do_reversed_fk(jv, *kinematic_chain[current_node->depth+1], current_node->tf);

		// Add as a child.
		tf_tree.children.emplace_back(jv, TfTreeNode {
			current_node->depth + 1,
			tf,
			{}
		});

//		std::vector<double> joint_values;
//
//		while (!current_node->children.empty()) {
//			joint_values.push_back(current_node->children[0].first);
//			current_node = &current_node->children[0].second;
//		}
//
//		// Mutate the first one randomly.
//		joint_values[0] = std::clamp(joint_values[0] + _rng.uniformReal(-0.1, 0.1), -M_PI / 2.0, M_PI / 2.0);
//
//		// Bring upright.
//
//		const size_t LOCKED_ANGLES = 1;
//
//		if (bringBaseUpright(tf_tree.tf, LOCKED_ANGLES, joint_values)) {
//
//			// Add as a child.
//			tf_tree.children.push_back(TfTreeNode {
//				1,
//			})
//
//		}



		// Repeatedly follow the first child.

//		const size_t LOCKED_ANGLES = 1;
//
//		joint_values[0] = std::clamp(joint_values[0] + 0.1, -M_PI / 2.0, M_PI / 2.0);
//
//		return bringBaseUpright(LOCKED_ANGLES, joint_values);

	}

};

int main() {

	const auto &robot = mgodpl::experiment_assets::loadRobotModel(1.0);

	const auto &tree_model = mgodpl::tree_meshes::loadTreeMeshes("appletree");

	math::Vec3d target(0.0, 0.0, 2.0);

	random_numbers::RandomNumberGenerator rng(43);

	GoalStateAlgorithm gsa(
			robot,
			tree_model.trunk_mesh,
			target,
			rng);


	mgodpl::SimpleVtkViewer viewer;
	viewer.addMesh(tree_model.trunk_mesh, WOOD_COLOR);
	viewer.addMesh(createGroundPlane(5.0, 5.0), FLOOR_COLOR);


	mgodpl::visualization::mkPointMarkerSphere(target, viewer)->GetProperty()->SetOpacity(0.5);

	{
		viewer.setCameraTransform(target + math::Vec3d(2.0, 1.0, 1.0), target);
	}

	bool algo_continue = true;

	std::vector<vtkSmartPointer<vtkActor>> link_actors;

	int timer = 10;

	viewer.addTimerCallback([&]() {

		if (timer-- == 0) {
			gsa.iterate();
			timer = 10;
		}


		// Delete old actors.
		for (auto &actor: link_actors) {
			viewer.viewerRenderer->RemoveActor(actor);
		}

		std::vector<const GoalStateAlgorithm::TfTreeNode*> stack = {&gsa.tf_tree};

		while (!stack.empty()) {
			const GoalStateAlgorithm::TfTreeNode *current_node = stack.back();
			stack.pop_back();

			const moveit::core::LinkModel *link = gsa.kinematic_chain[current_node->depth];

			if (!link->getCollisionOriginTransforms().empty()) {
				Eigen::Isometry3d transform = current_node->tf * link->getCollisionOriginTransforms()[0];

				auto link_actor = visualization::VtkRobotModel::actorForLink({0.8, 0.8, 0.8}, link);

				applyEigenTransformToActor(transform, link_actor);

				viewer.addActor(link_actor);
			}

			for (const auto& child: current_node->children) {
				stack.push_back(&child.second);
			}

		}

	});

//	viewer.startRecording("last_link_steady.ogv");

	viewer.start();

	std::cout << "Done." << std::endl;
}
