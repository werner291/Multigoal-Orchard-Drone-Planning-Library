//
// Created by werner on 30-9-22.
//

#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include "VtkRobotModel.h"
#include "../utilities/vtk.h"

vtkNew<vtkActorCollection> &VtkRobotmodel::getLinkActors() {
	return link_actors;
}

VtkRobotmodel::VtkRobotmodel(const moveit::core::RobotModelConstPtr &robot_model) : robotModel(robot_model) {

	for (const moveit::core::LinkModel* lm : robot_model->getLinkModelsWithCollisionGeometry()) {

		vtkNew<vtkActor> linkActor;

		vtkNew<vtkPolyDataMapper> linkPolyData = polyDataForLink(lm);

		linkActor->SetMapper(linkPolyData);

		linkActor->GetProperty()->SetColor(0.5, 0.5, 0.5);

		link_actors->AddItem(linkActor);

	}

}

void VtkRobotmodel::applyState(const moveit::core::RobotState &st) {
	for (size_t i = 0; i < link_actors->GetNumberOfItems(); ++i) {

		auto link_actor = vtkActor::SafeDownCast(link_actors->GetItemAsObject(i));
		auto lm = robotModel->getLinkModelsWithCollisionGeometry()[i];

		auto transform = lm->getCollisionOriginTransforms()[0];
		auto tf = st.getGlobalLinkTransform(lm);
		auto total_tf = tf * transform;

		Eigen::Vector3d tf_trans = total_tf.translation();
		link_actor->SetPosition(tf_trans.x(), tf_trans.y(), tf_trans.z());
		Eigen::AngleAxisd tf_rot(total_tf.rotation());
		link_actor->SetOrientation(0.0,0.0,0.0);
		link_actor->RotateWXYZ(tf_rot.angle() / M_PI * 180.0, tf_rot.axis().x(), tf_rot.axis().y(), tf_rot.axis().z());
	}
}

VtkRobotmodel::VtkRobotmodel(const moveit::core::RobotModelConstPtr &robot_model, const moveit::core::RobotState &st)
		: VtkRobotmodel(robot_model) {
	applyState(st);
}
