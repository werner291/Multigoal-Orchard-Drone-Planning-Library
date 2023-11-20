//
// Created by werner on 17-11-23.
//

#include <vtkProperty.h>
#include <moveit/robot_model/link_model.h>
#include <moveit/robot_model/robot_model.h>
#include <random_numbers/random_numbers.h>

#include "../experiment_utils/load_robot_model.h"
#include "../experiment_utils/mesh_utils.h"
#include "../planning/moveit_state_tools.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/VtkRobotModel.h"
#include "../visualization/quick_markers.h"
#include "../planning/JointSpacePoint.h"

using namespace mgodpl;

//if (ndofs_left == 1) {
//
//const auto &ideal_rotation = rotation_to_upright(tf.rotation());
//
//Eigen::Vector3d actual_axis =
//		tf * dynamic_cast<const moveit::core::RevoluteJointModel *>(parent_joint_model)->getAxis();
//double alignment = ideal_rotation.axis().dot(actual_axis);
//angle = -ideal_rotation.angle() * alignment;
//
//}

Eigen::AngleAxisd rotation_to_upright(const Eigen::Isometry3d::RotationReturnType& current_tf) {
	Eigen::Vector3d local_up = current_tf * Eigen::Vector3d(0.0, 0.0, 1.0);

	Eigen::Vector3d ideal_axis = local_up.cross(Eigen::Vector3d(0.0, 0.0, 1.0)).normalized();

	double ideal_angle = std::acos(std::clamp(local_up.dot(Eigen::Vector3d(0.0, 0.0, 1.0)),
											  -1.0,
											  1.0));

	Eigen::AngleAxisd ideal_rotation(ideal_angle, ideal_axis);

	return ideal_rotation;
}

std::vector<Eigen::Isometry3d> do_reversed_fk(
		const std::vector<double> &joint_values,
		const std::vector<const moveit::core::LinkModel *> &kinematic_chain,
		const Eigen::Isometry3d &end_effector_tf) {

	std::vector<Eigen::Isometry3d> link_tfs;
	link_tfs.reserve(kinematic_chain.size());

	link_tfs.push_back(end_effector_tf);

	int joint_value_index = 0;

	for (size_t i = 1; i < kinematic_chain.size(); ++i) {
		auto parent_joint_model = kinematic_chain[i - 1]->getParentJointModel();

		auto joint = parent_joint_model->getType();

		switch (joint) {
			case moveit::core::JointModel::FIXED: {
				link_tfs.push_back(link_tfs.back() * kinematic_chain[i - 1]->getJointOriginTransform().inverse());
			}
				break;

			case moveit::core::JointModel::REVOLUTE: {
				double angle = joint_values[joint_value_index++];

				Eigen::Isometry3d joint_tf;
				parent_joint_model->computeTransform(&angle, joint_tf);

				link_tfs.push_back(link_tfs.back() * joint_tf.inverse() *
								   kinematic_chain[i - 1]->getJointOriginTransform().inverse());
			}

				break;

			default:
				throw std::runtime_error("Unsupported joint type.");

		}
	}

	return link_tfs;
}

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

int main() {
	const auto &robot = mgodpl::experiment_assets::loadRobotModel(1.0);

	math::Vec3d target(0.0, 0.0, 2.0);

	mgodpl::SimpleVtkViewer viewer;

	random_numbers::RandomNumberGenerator rng(42);

	moveit_facade::JointSpacePoint gs = experiment_state_tools::genGoalSampleUniform(target, rng, *robot);
//
//	mgodpl::visualization::VtkRobotModel robot_model(robot, gs, {0.5, 0.5, 0.5});
//	viewer.addActorCollection(robot_model.getLinkActors());

	std::vector<const moveit::core::LinkModel *> kinematic_chain{robot->getLinkModel("end_effector")};

	while (kinematic_chain.back()->getParentLinkModel() != nullptr) {
		kinematic_chain.push_back(kinematic_chain.back()->getParentLinkModel());
	}

	// So, the translation of the end-effector is fixed: it's the target point.
	// The rotation must now be chosen such that the base link can still be upright by determining the variables of the other joints.

	// Pick a random rotation axis.
	Eigen::Vector3d axis(rng.gaussian01(), rng.gaussian01(), rng.gaussian01());
	axis.normalize();

	// For a test, pick a random rotation.
	// Eigen::Isometry3d tf = Eigen::Translation3d(target.x, target.y, target.z);
	Eigen::AngleAxisd rotation(rng.uniformReal(-M_PI, M_PI), axis);

	Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
	tf *= Eigen::Translation3d(target.x(), target.y(), target.z());
	tf *= rotation;

	vtkSmartPointer<vtkActor> ee_actor =
			visualization::VtkRobotModel::actorForLink({0.8, 0.8, 0.8}, kinematic_chain[0]);

	ee_actor->SetPosition(target.x(), target.y(), target.z());
	ee_actor->SetOrientation(0.0, 0.0, 0.0);
	ee_actor->RotateWXYZ(rotation.angle() / M_PI * 180.0,
						 rotation.axis().x(),
						 rotation.axis().y(),
						 rotation.axis().z());

	viewer.addActor(ee_actor);

	std::vector<double> joint_values(kinematic_chain.size() - 2, 0.0);

	std::vector<Eigen::Isometry3d> link_tfs = do_reversed_fk(joint_values, kinematic_chain, tf);

	// Compute ideal rotation to get the base upright.
	Eigen::AngleAxisd ideal_rotation = rotation_to_upright(link_tfs.back().rotation());

	std::vector<vtkSmartPointer<vtkActor>> link_actors;
	for (size_t i1 = 1; i1 < kinematic_chain.size(); ++i1) {
		link_actors.push_back(visualization::VtkRobotModel::actorForLink({0.8, 0.8, 0.8}, kinematic_chain[i1]));
		viewer.addActor(link_actors.back());
	}

	for (size_t i1 = 1; i1 < kinematic_chain.size(); ++i1) {

		Eigen::Isometry3d transform = link_tfs[i1] * kinematic_chain[i1]->getCollisionOriginTransforms()[0];

		applyEigenTransformToActor(transform, link_actors[i1-1]);

	}

	math::Vec3d tf_center(tf.translation().x(), tf.translation().y(), tf.translation().z());

	mgodpl::visualization::mkPointMarkerSphere(target, viewer)->GetProperty()->SetOpacity(0.5);

	viewer.addMesh(createGroundPlane(5.0, 5.0), {0.5, 0.3, 0.1});

	{
		math::Vec3d mean_center { 0.0, 0.0, 0.0 };
		for (const auto &link_tf: link_tfs) {
			mean_center = mean_center + math::Vec3d(link_tf.translation().x(), link_tf.translation().y(), link_tf.translation().z());
		}
		mean_center = mean_center / (double) link_tfs.size();
		viewer.setCameraTransform(mean_center + math::Vec3d(2.0, 1.0, 1.0), mean_center);
	}

	viewer.addTimerCallback([&]() {

		ideal_rotation = rotation_to_upright(link_tfs.back().rotation());

		const size_t LOCKED_ANGLES = 1;

		joint_values[0] = std::clamp(joint_values[0] + 0.01, -M_PI / 2.0, M_PI / 2.0);

		for (size_t i = 1 + LOCKED_ANGLES; i < joint_values.size(); ++i) {
			Eigen::Vector3d axis = dynamic_cast<const moveit::core::RevoluteJointModel *>(kinematic_chain[i+1]->getParentJointModel())->getAxis();
			Eigen::Vector3d actual_axis = link_tfs[i+1].rotation() * axis;
			double alignment = ideal_rotation.axis().dot(actual_axis);

			joint_values[i] -= std::clamp(alignment * ideal_rotation.angle(), -0.1, 0.1);
		}

		link_tfs = do_reversed_fk(joint_values, kinematic_chain, tf);

		for (size_t i1 = 1; i1 < kinematic_chain.size(); ++i1) {

			Eigen::Isometry3d transform = link_tfs[i1] * kinematic_chain[i1]->getCollisionOriginTransforms()[0];

			applyEigenTransformToActor(transform, link_actors[i1-1]);

		}

	});

//	viewer.startRecording("last_link_steady.ogv");

	viewer.start();

	std::cout << "Done." << std::endl;
}
