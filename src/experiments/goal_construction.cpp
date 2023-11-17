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

using namespace mgodpl;

int main()
{
    const auto& robot = mgodpl::experiment_assets::loadRobotModel(1.0);

    math::Vec3d target(0.0, 0.0, 2.0);

    mgodpl::SimpleVtkViewer viewer;

    random_numbers::RandomNumberGenerator rng(42);

    std::vector<const moveit::core::LinkModel*> kinematic_chain{robot->getLinkModel("end_effector")};

    while (kinematic_chain.back()->getParentLinkModel() != nullptr)
    {
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
        visualization::VtkRobotModel::actorForLink({1.0, 0.0, 0.0}, kinematic_chain[0]);

    ee_actor->SetPosition(target.x(), target.y(), target.z());
    ee_actor->SetOrientation(0.0, 0.0, 0.0);
    ee_actor->RotateWXYZ(rotation.angle() / M_PI * 180.0,
                         rotation.axis().x(),
                         rotation.axis().y(),
                         rotation.axis().z());

    viewer.addActor(ee_actor);

    for (size_t i = 1; i < kinematic_chain.size(); ++i)
    {
        auto parent_joint_model = kinematic_chain[i - 1]->getParentJointModel();

        auto joint = parent_joint_model->getType();

        vtkSmartPointer<vtkActor> link_actor = visualization::VtkRobotModel::actorForLink(
                    {1.0, 0.0, 0.0}, kinematic_chain[i]);

       int ndofs_left = kinematic_chain.size() - i;

        if (ndofs_left > 2)
        {
            switch (joint)
            {
            case moveit::core::JointModel::FIXED:
                {
                    tf = tf * kinematic_chain[i - 1]->getJointOriginTransform().inverse();
                }
                break;

            case moveit::core::JointModel::REVOLUTE:
                {
                    double angle = rng.uniformReal(-1.0,1.0);

                    if (ndofs_left == 1)
                    {

                        Eigen::Vector3d local_up = tf.rotation() * Eigen::Vector3d(0.0, 0.0, 1.0);

                        Eigen::Vector3d ideal_axis = local_up.cross(Eigen::Vector3d(0.0, 0.0, 1.0)).normalized();
                        Eigen::Vector3d actual_axis = tf * dynamic_cast<const moveit::core::RevoluteJointModel*>(parent_joint_model)->getAxis();

                        double alignment = ideal_axis.dot(actual_axis);

                        double ideal_angle = std::acos(std::clamp(local_up.dot(Eigen::Vector3d(0.0, 0.0, 1.0)), -1.0, 1.0));

                        angle = -ideal_angle * alignment;

                    }

                    Eigen::Isometry3d joint_tf;
                    parent_joint_model->computeTransform(&angle, joint_tf);

                    tf = tf * joint_tf.inverse() * kinematic_chain[i - 1]->getJointOriginTransform().inverse();
                }

                break;

            default:
                throw std::runtime_error("Unsupported joint type.");
            }
        } else {
            // Make sure the last two joints are revolute.
            assert(
                        kinematic_chain[i]->getParentJointModel()->getType() == moveit::core::JointModel::REVOLUTE &&
                        kinematic_chain[i+1]->getParentJointModel()->getType() == moveit::core::JointModel::REVOLUTE
            );

            // Make sure the axes are linearly independent.
            assert(
                  dynamic_cast<const moveit::core::RevoluteJointModel*>(kinematic_chain[i]->getParentJointModel())->getAxis().cross(
                      dynamic_cast<const moveit::core::RevoluteJointModel*>(kinematic_chain[i+1]->getParentJointModel())->getAxis()
                  ).norm() > 1e-6
            );


        }

         Eigen::Isometry3d transform = tf * kinematic_chain[i]->getCollisionOriginTransforms()[0];

        link_actor->SetPosition(
                    transform.translation().x(),
                    transform.translation().y(),
                    transform.translation().z());

        Eigen::AngleAxisd tf_rot(transform.rotation());

        link_actor->SetOrientation(0.0, 0.0, 0.0);

        link_actor->RotateWXYZ(tf_rot.angle() / M_PI * 180.0,
                               tf_rot.axis().x(),
                               tf_rot.axis().y(),
                               tf_rot.axis().z());

        viewer.addActor(link_actor);
    }

    math::Vec3d tf_center(tf.translation().x(), tf.translation().y(), tf.translation().z());

    mgodpl::visualization::mkPointMarkerSphere(target, viewer)->GetProperty()->SetOpacity(0.5);

    viewer.addMesh(createGroundPlane(5.0,5.0), {0.5,0.3,0.1});

    viewer.lockCameraUp();

    viewer.start();

    std::cout << "Done." << std::endl;
}
