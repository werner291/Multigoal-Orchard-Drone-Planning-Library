// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <utility>

#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometric_shapes/shapes.h>
#include <vtkSphereSource.h>
#include <vtkCylinderSource.h>
#include <vtkCubeSource.h>

#include "VtkRobotModel.h"
#include "../experiment_utils/load_mesh_ros.h"
#include "vtk.h"

vtkNew<vtkPolyDataMapper> polyDataForLink(const moveit::core::LinkModel *lm) {

	// Allocate a new polydata mapper
	vtkNew<vtkPolyDataMapper> linkPolyData;

	// Get the first shape of the link (assuming it has one)
	assert(lm->getShapes().size() >= 1);
	auto shape = lm->getShapes()[0];

	// Get the shape type and jump to the appropriate case
	switch (shape->type) {

		case shapes::SPHERE: {
			auto sphere = std::dynamic_pointer_cast<const shapes::Sphere>(shape);
			vtkNew<vtkSphereSource> sphereSource;
			sphereSource->SetRadius(sphere->radius);
			linkPolyData->SetInputConnection(sphereSource->GetOutputPort());
		}
			break;
		case shapes::CYLINDER: {
			auto cylinder = std::dynamic_pointer_cast<const shapes::Cylinder>(shape);
			vtkNew<vtkCylinderSource> cylinderSource;
			cylinderSource->SetRadius(cylinder->radius);
			cylinderSource->SetHeight(cylinder->length);
			linkPolyData->SetInputConnection(cylinderSource->GetOutputPort());
		}
			break;
		case shapes::CONE:
			throw std::runtime_error("Cone shape type not supported");
			break;
		case shapes::BOX: {
			auto box = std::dynamic_pointer_cast<const shapes::Box>(shape);
			vtkNew<vtkCubeSource> cubeSource;
			cubeSource->SetXLength(box->size[0]);
			cubeSource->SetYLength(box->size[1]);
			cubeSource->SetZLength(box->size[2]);

			linkPolyData->SetInputConnection(cubeSource->GetOutputPort());
		}
			break;
		case shapes::PLANE:
			throw std::runtime_error("Plane shape type not supported");
			break;
		case shapes::MESH:
			throw std::runtime_error("Mesh shape type not supported");
			break;
		case shapes::OCTREE:
			throw std::runtime_error("Octree shape type not supported");
			break;

		default:
		case shapes::UNKNOWN_SHAPE: {
			throw std::runtime_error("Unknown shape type");
			break;
		}
	}

	return linkPolyData;
}

namespace mgodpl::visualization {

	vtkNew<vtkActorCollection> &VtkRobotModel::getLinkActors() {
		return link_actors;
	}

	VtkRobotModel::VtkRobotModel(moveit::core::RobotModelConstPtr robot_model,
								 const moveit::core::RobotState &initial_state,
								 const math::Vec3d &rgb) : robotModel(std::move(robot_model)) {

		generateLinkActors(rgb);

		applyState(initial_state);

	}

	void
	VtkRobotModel::generateLinkActors(const math::Vec3d &rgb) {
		for (const moveit::core::LinkModel *lm: robotModel->getLinkModelsWithCollisionGeometry()) {

			std::cout << "Visualizing link with mesh " << lm->getVisualMeshFilename() << std::endl;

			if (!lm->getVisualMeshFilename().empty()) {

				auto mesh = loadRobotMesh(lm->getVisualMeshFilename());

				auto actor = createActorFromMesh(mesh);

				actor->GetProperty()->SetColor(rgb.x(), rgb.y(), rgb.z());

				link_actors->AddItem(actor);

			} else {

				vtkNew<vtkActor> linkActor;

				vtkNew<vtkPolyDataMapper> linkPolyData = polyDataForLink(lm);

				linkActor->SetMapper(linkPolyData);

				linkActor->GetProperty()->SetColor(rgb.x(), rgb.y(), rgb.z());

				link_actors->AddItem(linkActor);
			}

		}
	}

	void VtkRobotModel::applyState(const moveit::core::RobotState &st) {
		for (size_t i = 0; i < link_actors->GetNumberOfItems(); ++i) {

			auto link_actor = vtkActor::SafeDownCast(link_actors->GetItemAsObject(i));
			auto lm = robotModel->getLinkModelsWithCollisionGeometry()[i];

			auto transform = lm->getCollisionOriginTransforms()[0];
			auto tf = st.getGlobalLinkTransform(lm);
			auto total_tf = tf * transform;

			Eigen::Vector3d tf_trans = total_tf.translation();
			link_actor->SetPosition(tf_trans.x(), tf_trans.y(), tf_trans.z());
			Eigen::AngleAxisd tf_rot(total_tf.rotation());
			link_actor->SetOrientation(0.0, 0.0, 0.0);
			link_actor->RotateWXYZ(tf_rot.angle() / M_PI * 180.0,
								   tf_rot.axis().x(),
								   tf_rot.axis().y(),
								   tf_rot.axis().z());
		}
	}

}