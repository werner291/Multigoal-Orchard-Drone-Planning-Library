#include <geometric_shapes/shapes.h>
#include <vtkSphereSource.h>
#include <vtkCylinderSource.h>
#include <vtkCubeSource.h>
#include <vtkRenderer.h>
#include <vtkRendererSource.h>
#include <vtkRenderWindow.h>

#include <vtkRenderWindowInteractor.h>
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

vtkNew<vtkLight> mkWhiteAmbientLight() {
	vtkNew<vtkLight> light;
	light->SetDiffuseColor(0.0,0.0,0.0);
	light->SetAmbientColor(1.0,1.0,1.0);
	light->SetLightTypeToSceneLight();
	return light;
}

void setCameraFromEigen(Eigen::Isometry3d &tf, vtkCamera *pCamera) {
	Eigen::Vector3d eye_center = tf.translation();
	Eigen::Vector3d eye_focus = tf * Eigen::Vector3d(0, 1.0, 0.0);
	Eigen::Vector3d eye_up = tf.rotation() * Eigen::Vector3d(0, 0.0, 1.0);

	pCamera->SetPosition(eye_center.x(), eye_center.y(), eye_center.z());
	pCamera->SetFocalPoint(eye_focus.x(), eye_focus.y(), eye_focus.z());
	pCamera->SetViewUp(eye_up.x(), eye_up.y(), eye_up.z());
}

void addActorCollectionToRenderer(vtkNew<vtkActorCollection> &orchard_actors, vtkNew<vtkRenderer> &sensorRenderer) {
	for (int i = 0; i < orchard_actors->GetNumberOfItems(); i++) {
		sensorRenderer->AddActor(vtkActor::SafeDownCast(orchard_actors->GetItemAsObject(i)));
	}
}

vtkNew<vtkCellArray> meshTrianglesToVtkCells(const shape_msgs::msg::Mesh &mesh) {
	vtkNew<vtkCellArray> cells;
	for (auto &triangle : mesh.triangles) {
		cells->InsertNextCell({
									  triangle.vertex_indices[0],
									  triangle.vertex_indices[1],
									  triangle.vertex_indices[2]
							  });
	}
	return cells;
}

vtkNew<vtkPoints> meshVerticesToVtkPoints(const shape_msgs::msg::Mesh &mesh) {
	vtkNew<vtkPoints> points;
	for (auto &point : mesh.vertices) {
		points->InsertNextPoint(point.x, point.y, point.z);
	}
	return points;
}

vtkNew<vtkPolyData> rosMeshToVtkPolyData(const shape_msgs::msg::Mesh &mesh) {

	vtkNew<vtkPoints> points = meshVerticesToVtkPoints(mesh);

	vtkNew<vtkCellArray> cells = meshTrianglesToVtkCells(mesh);

	vtkNew<vtkPolyData> polyData;

	polyData->SetPoints(points);
	polyData->SetPolys(cells);

	return polyData;

}

vtkNew<vtkActor> createActorFromMesh(const shape_msgs::msg::Mesh &mesh) {

	vtkNew<vtkPolyData> polyData = rosMeshToVtkPolyData(mesh);

	vtkNew<vtkPolyDataMapper> mapper;
	mapper->SetInputData(polyData);

	vtkNew<vtkActor> actor;
	actor->SetMapper(mapper);

	return actor;
}

vtkNew<vtkDepthImageToPointCloud> extractPointCloudFromRenderer(vtkNew<vtkRenderer> &sensorRenderer) {
	vtkNew<vtkRendererSource> rendererSource;
	rendererSource->DepthValuesOn();
	rendererSource->SetInput(sensorRenderer);

	vtkNew<vtkDepthImageToPointCloud> depthToPointCloud;
	depthToPointCloud->SetInputConnection(0, rendererSource->GetOutputPort());
	depthToPointCloud->SetCamera(sensorRenderer->GetActiveCamera());
	return depthToPointCloud;
}

vtkFunctionalCallback *vtkFunctionalCallback::New() {
	return new vtkFunctionalCallback;
}

void vtkFunctionalCallback::Execute(vtkObject *caller, unsigned long eventId, void *) {
	if (eventId == event_id)
	{
		callback();
	}
}

void vtkFunctionalCallback::setEventId(unsigned long eventId) {
	event_id = eventId;
}
