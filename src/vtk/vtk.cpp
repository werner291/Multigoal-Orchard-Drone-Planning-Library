#include <geometric_shapes/shapes.h>
#include <vtkSphereSource.h>
#include <vtkCylinderSource.h>
#include <vtkCubeSource.h>
#include "vtk.h"

vtkNew <vtkPolyDataMapper> polyDataForLink(const moveit::core::LinkModel *lm) {
	vtkNew<vtkPolyDataMapper> linkPolyData;

	auto shape = lm->getShapes()[0];

	switch (shape->type) {

		case shapes::UNKNOWN_SHAPE: {
			throw std::runtime_error("Unknown shape type");
			break;
		}

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
	}
	return linkPolyData;
}
