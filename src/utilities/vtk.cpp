#include <geometric_shapes/shapes.h>
#include <vtkSphereSource.h>
#include <vtkCylinderSource.h>
#include <vtkCubeSource.h>
#include <vtkRenderer.h>
#include <vtkRendererSource.h>
#include <vtkRenderWindow.h>
#include <vtkProperty.h>
#include <vtkPointData.h>

#include <vtkRenderWindowInteractor.h>
#include <vtkPlaneSource.h>
#include "vtk.h"
#include "../exploration/ColorEncoding.h"

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

void setCameraFromEigen(const Eigen::Isometry3d &tf, vtkCamera *pCamera) {
	Eigen::Vector3d eye_center = tf.translation();
	Eigen::Vector3d eye_focus = tf * Eigen::Vector3d(0, 1.0, 0.0);
	Eigen::Vector3d eye_up = tf.rotation() * Eigen::Vector3d(0, 0.0, 1.0);

	pCamera->SetPosition(eye_center.x(), eye_center.y(), eye_center.z());
	pCamera->SetFocalPoint(eye_focus.x(), eye_focus.y(), eye_focus.z());
	pCamera->SetViewUp(eye_up.x(), eye_up.y(), eye_up.z());
}

void addActorCollectionToRenderer(vtkActorCollection *orchard_actors, vtkRenderer *sensorRenderer) {
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
	mapper->SetColorModeToDefault();

	vtkNew<vtkActor> actor;
	actor->SetMapper(mapper);

	return actor;
}


vtkNew<vtkPolyData> mkVtkPolyDataFromScannablePoints(const std::vector<ScanTargetPoint> &scan_targets) {
	vtkNew<vtkUnsignedCharArray> colors;
	colors->SetNumberOfComponents(3);

	vtkNew<vtkPoints> fruitSurfacePointsVtk;
	vtkNew<vtkCellArray> fruitSurfaceCells;

	for (const auto &point: scan_targets) {
		auto pt_id = fruitSurfacePointsVtk->InsertNextPoint(point.point.data());
		fruitSurfaceCells->InsertNextCell({pt_id});

		colors->InsertNextTuple3(0, 0, 255);
	}

	vtkNew<vtkPolyData> fruitSurfacePolyData;
	fruitSurfacePolyData->SetPoints(fruitSurfacePointsVtk);
	fruitSurfacePolyData->SetVerts(fruitSurfaceCells);
	fruitSurfacePolyData->GetPointData()->SetScalars(colors);
	return fruitSurfacePolyData;
}

vtkNew<vtkRenderer> buildSensorRenderer() {

	vtkNew<vtkRenderer> sensorRenderer;
	sensorRenderer->SetBackground(0.0, 0.0, 0.0);
	sensorRenderer->GetActiveCamera()->SetClippingRange(0.1, 10.0);

	vtkNew<vtkLight> solidColorLight = mkWhiteAmbientLight();
	sensorRenderer->RemoveAllLights();
	sensorRenderer->AddLight(solidColorLight);

	return sensorRenderer;
}

vtkNew<vtkRenderWindow> buildSensorRenderWindow(vtkRenderer *sensorRenderer) {
	vtkNew<vtkRenderWindow> sensorViewWindow;
	sensorViewWindow->SetSize(SENSOR_RESOLUTION, SENSOR_RESOLUTION);
	sensorViewWindow->AddRenderer(sensorRenderer);
	sensorViewWindow->SetWindowName("Robot Sensor View");
	return sensorViewWindow;
}

void setColorsByEncoding(vtkNew<vtkActor> &tree_actor, const std::array<double, 3> &rgb, bool usePureColor) {
	tree_actor->GetProperty()->SetDiffuseColor(rgb.data());
	tree_actor->GetProperty()->SetAmbientColor(rgb.data());

	if (usePureColor) {
		tree_actor->GetProperty()->SetAmbient(1.0);
	} else {
		tree_actor->GetProperty()->SetAmbient(0.2);
		tree_actor->GetProperty()->SetDiffuse(0.8);
	}
}

vtkNew<vtkActorCollection> buildTreeActors(const TreeMeshes &meshes, bool usePureColor) {

	vtkNew<vtkActorCollection> actors;

	auto tree_actor = createActorFromMesh(meshes.trunk_mesh);
	setColorsByEncoding(tree_actor, TRUNK_RGB, usePureColor);

	auto leaves_actor = createActorFromMesh(meshes.leaves_mesh);
	setColorsByEncoding(leaves_actor, LEAVES_RGB, usePureColor);

	actors->AddItem(tree_actor);
	actors->AddItem(leaves_actor);

	for (const auto& fruit_mesh : meshes.fruit_meshes) {
		auto fruit_actor = createActorFromMesh(fruit_mesh);
		setColorsByEncoding(fruit_actor, FRUIT_RGB, usePureColor);
		actors->AddItem(fruit_actor);
	}

	return actors;
}

vtkNew<vtkActorCollection> buildOrchardActors(const SimplifiedOrchard &orchard, bool usePureColor) {

	vtkNew<vtkActorCollection> orchard_actors;

	for (const auto& [pos, meshes] : orchard.trees)
	{
		auto tree_actors = buildTreeActors(meshes, usePureColor);

		for (int i = 0; i < tree_actors->GetNumberOfItems(); i++) {
			auto actor = vtkActor::SafeDownCast(tree_actors->GetItemAsObject(i));
			actor->SetPosition(pos.x(), pos.y(), 0);

			orchard_actors->AddItem(actor);

		}
	}

	vtkNew<vtkActor> ground_plane_actor = buildGroundPlaneActor(usePureColor);

	orchard_actors->AddItem(ground_plane_actor);

	return orchard_actors;

}

vtkNew<vtkActor> buildGroundPlaneActor(bool usePureColor) {
	vtkNew<vtkPlaneSource> ground_plane_source;
	ground_plane_source->SetPoint1(10.0, 0.0, 0.0);
	ground_plane_source->SetPoint2(0.0, 10.0, 0.0);
	ground_plane_source->SetCenter(0.0, 0.0, 0.0);

	vtkNew<vtkPolyDataMapper> ground_plane_mapper;
	ground_plane_mapper->SetInputConnection(ground_plane_source->GetOutputPort());

	vtkNew<vtkActor> ground_plane_actor;
	ground_plane_actor->SetMapper(ground_plane_mapper);

	setColorsByEncoding(ground_plane_actor, GROUND_PLANE_RGB, usePureColor);

	return ground_plane_actor;
}

vtkNew<vtkActor> buildDepthImagePointCloudActor(vtkAlgorithmOutput *pointCloudInput) {
	vtkNew<vtkPolyDataMapper> pointCloudMapper;
	pointCloudMapper->SetInputConnection(pointCloudInput);

	vtkNew<vtkActor> pointCloudActor;
	pointCloudActor->SetMapper(pointCloudMapper);
	return pointCloudActor;
}

vtkSmartPointer<vtkActor> addColoredMeshActor(const shape_msgs::msg::Mesh &mesh,
											  const std::array<double, 4> &color_rgba,
											  vtkRenderer *renderer,
											  bool visible) {

	auto actor = createColoredMeshActor(mesh, color_rgba, visible);
	renderer->AddActor(actor);
	return actor;

}

vtkSmartPointer<vtkActor>
createColoredMeshActor(const shape_msgs::msg::Mesh &mesh, const std::array<double, 4> &color_rgba, bool visible) {

	auto actor = createActorFromMesh(mesh);
	actor->GetProperty()->SetColor(color_rgba[0], color_rgba[1], color_rgba[2]);
	if (color_rgba[3] < 1.0) {
		actor->GetProperty()->SetOpacity(color_rgba[3]);
	}
	return actor;

}

vtkFunctionalCallback *vtkFunctionalCallback::New() {
	return new vtkFunctionalCallback;
}

void vtkFunctionalCallback::Execute(vtkObject *caller, unsigned long eventId, void *) {
	if (eventId == event_id) {
		callback();
	}
}

void vtkFunctionalCallback::setEventId(unsigned long eventId) {
	event_id = eventId;
}

VtkPolyLineVisualization::VtkPolyLineVisualization(float r, float g, float b) {
	visitOrderVisualizationMapper->SetInputData(visitOrderVisualizationData);
	visitOrderVisualizationActor->SetMapper(visitOrderVisualizationMapper);
	visitOrderVisualizationActor->GetProperty()->SetColor(r,g,b);
	visitOrderVisualizationActor->GetProperty()->SetLineWidth(5);
	visitOrderVisualizationActor->GetProperty()->SetPointSize(8);

}

vtkActor *VtkPolyLineVisualization::getActor() {
	return visitOrderVisualizationActor;
}

void VtkPolyLineVisualization::updateLine(const std::vector<Eigen::Vector3d> &points) {

	assert(!points.empty());

	vtkNew<vtkPoints> pointsVtk;
	vtkNew<vtkCellArray> cells;

	vtkNew<vtkCellArray> pointsCells;

	auto previousPointId = pointsVtk->InsertNextPoint(points[0].data());

	pointsCells->InsertNextCell(1);
	pointsCells->InsertCellPoint(previousPointId);

	for (size_t i = 1; i < points.size(); ++i) {
		cells->InsertNextCell(2);
		cells->InsertCellPoint(previousPointId);
		cells->InsertCellPoint(previousPointId = pointsVtk->InsertNextPoint(points[i].data()));

		pointsCells->InsertNextCell(1);
		pointsCells->InsertCellPoint(previousPointId);
	}

	visitOrderVisualizationData->SetPoints(pointsVtk);
	visitOrderVisualizationData->SetLines(cells);
	visitOrderVisualizationData->SetVerts(pointsCells);

	visitOrderVisualizationData->Modified();
}

VtkLineSegmentsVisualization::VtkLineSegmentsVisualization(float r, float g, float b) {
	visitOrderVisualizationMapper->SetInputData(visitOrderVisualizationData);
	visitOrderVisualizationActor->SetMapper(visitOrderVisualizationMapper);
	visitOrderVisualizationActor->GetProperty()->SetColor(r, g, b);
	visitOrderVisualizationActor->GetProperty()->SetLineWidth(2);
}

vtkActor *VtkLineSegmentsVisualization::getActor() {
	return visitOrderVisualizationActor;
}

void VtkLineSegmentsVisualization::updateLine(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &points) {

	vtkNew<vtkPoints> pointsVtk;
	vtkNew<vtkCellArray> cells;

	for (const auto& [p1, p2] : points) {
		cells->InsertNextCell(2);
		cells->InsertCellPoint(pointsVtk->InsertNextPoint(p1.data()));
		cells->InsertCellPoint(pointsVtk->InsertNextPoint(p2.data()));
	}

	visitOrderVisualizationData->SetPoints(pointsVtk);
	visitOrderVisualizationData->SetLines(cells);

	visitOrderVisualizationData->Modified();

}

VtkPointCloudVisualization::VtkPointCloudVisualization(float r, float g, float b) {
	visitOrderVisualizationMapper->SetInputData(visitOrderVisualizationData);
	visitOrderVisualizationActor->SetMapper(visitOrderVisualizationMapper);
	visitOrderVisualizationActor->GetProperty()->SetColor(r, g, b);
	visitOrderVisualizationActor->GetProperty()->SetPointSize(3);
}

vtkActor *VtkPointCloudVisualization::getActor() {
	return visitOrderVisualizationActor;
}

void VtkPointCloudVisualization::updatePoints(const std::vector<Eigen::Vector3d> &points) {

	assert(!points.empty());

	vtkNew<vtkPoints> pointsVtk;
	vtkNew<vtkCellArray> cells;

	for (const auto &p: points) {
		cells->InsertNextCell(1);
		cells->InsertCellPoint(pointsVtk->InsertNextPoint(p.data()));
	}

	visitOrderVisualizationData->SetPoints(pointsVtk);
	visitOrderVisualizationData->SetVerts(cells);

	visitOrderVisualizationData->Modified();

}
