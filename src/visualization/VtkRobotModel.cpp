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


namespace mgodpl::visualization {

	vtkNew<vtkPolyDataMapper> polyDataForLink(const moveit::core::LinkModel* lm)
	{

		// Allocate a new polydata mapper
		vtkNew<vtkPolyDataMapper> linkPolyData;

		if (lm->getName() == "end_effector") { // Hacky, but it works. For some reason, non-mesh visual geometries are not stored in the link model.
			vtkNew<vtkCubeSource> cubeSource;
			cubeSource->SetXLength(0.08);
			cubeSource->SetYLength(0.08);
			cubeSource->SetZLength(0.08);

			linkPolyData->SetInputConnection(cubeSource->GetOutputPort());

			return linkPolyData;
		}

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

	vtkNew<vtkActorCollection> &VtkRobotModel::getLinkActors() {
		return link_actors;
	}

	VtkRobotModel::VtkRobotModel(moveit::core::RobotModelConstPtr robot_model,
								 const moveit_facade::JointSpacePoint &initial_state,
								 const math::Vec3d &rgb) : robotModel(std::move(robot_model)) {

		generateLinkActors(rgb);

		applyState(initial_state);

	}

	vtkSmartPointer<vtkActor> VtkRobotModel::actorForLink(const math::Vec3d& rgb, const moveit::core::LinkModel* lm)
	{
		if (!lm->getVisualMeshFilename().empty()) {

			auto mesh = loadRobotMesh(lm->getVisualMeshFilename());

			vtkSmartPointer<vtkActor> actor = createActorFromMesh(mesh);

			actor->GetProperty()->SetColor(rgb.x(), rgb.y(), rgb.z());

			return actor;

		} else {

			vtkNew<vtkActor> linkActor;

			vtkNew<vtkPolyDataMapper> linkPolyData = polyDataForLink(lm);

			linkActor->SetMapper(linkPolyData);

			linkActor->GetProperty()->SetColor(rgb.x(), rgb.y(), rgb.z());

			return linkActor;
		}
	}

	void VtkRobotModel::generateLinkActors(const math::Vec3d &rgb) {
		for (const moveit::core::LinkModel *lm: robotModel->getLinkModelsWithCollisionGeometry()) {

			std::cout << "Visualizing link with mesh " << lm->getVisualMeshFilename() << std::endl;

			vtkSmartPointer<vtkActor> actor;
			actorForLink(rgb, lm);

			this->link_actors->AddItem(actor);

		}
	}

	void VtkRobotModel::applyState(const moveit_facade::JointSpacePoint &st) {

		moveit::core::RobotState moveit_state(robotModel);
		st.to_moveit(moveit_state, true);

		for (size_t i = 0; i < (size_t) link_actors->GetNumberOfItems(); ++i) {

			auto link_actor = vtkActor::SafeDownCast(link_actors->GetItemAsObject(i));
			auto lm = robotModel->getLinkModelsWithCollisionGeometry()[i];

			// auto transform = lm->getCollisionOriginTransforms()[0];
			// auto tf = moveit_state.getGlobalLinkTransform(lm);
			// auto total_tf = tf * transform;
			auto total_tf = moveit_state.getCollisionBodyTransform(lm, 0);

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