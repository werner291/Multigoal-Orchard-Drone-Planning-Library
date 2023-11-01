#include <geometric_shapes/shapes.h>
#include <vtkSphereSource.h>
#include <vtkCylinderSource.h>
#include <vtkCubeSource.h>
#include <vtkRenderer.h>
#include <vtkRendererSource.h>
#include <vtkRenderWindow.h>
#include <vtkProperty.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkLight.h>
#include <vtkCamera.h>

#include <range/v3/view/transform.hpp>
#include <range/v3/to_container.hpp>

#include <vtkRenderWindowInteractor.h>
#include <vtkPlaneSource.h>
#include "vtk.h"

vtkNew<vtkLight> mkWhiteAmbientLight() {
	vtkNew<vtkLight> light;
	light->SetDiffuseColor(0.0,0.0,0.0);
	light->SetAmbientColor(1.0,1.0,1.0);
	light->SetLightTypeToSceneLight();
	return light;
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

vtkNew<vtkActorCollection> buildTreeActors(const mgodpl::tree_meshes::TreeMeshes &meshes, bool usePureColor) {

	vtkNew<vtkActorCollection> actors;

	auto tree_actor = createActorFromMesh(meshes.trunk_mesh);
	setColorsByEncoding(tree_actor, {0.5, 0.3, 0.1}, usePureColor);

	auto leaves_actor = createActorFromMesh(meshes.leaves_mesh);
	setColorsByEncoding(leaves_actor, {0.0, 0.9, 0.0}, usePureColor);

	actors->AddItem(tree_actor);
	actors->AddItem(leaves_actor);

	for (const auto& fruit_mesh : meshes.fruit_meshes) {
		auto fruit_actor = createActorFromMesh(fruit_mesh);
		setColorsByEncoding(fruit_actor, {1.0, 0.0, 0.0}, usePureColor);
		actors->AddItem(fruit_actor);
	}

	return actors;
}

vtkNew<vtkActorCollection> buildOrchardActors(const mgodpl::tree_meshes::SimplifiedOrchard &orchard, bool usePureColor) {

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

	setColorsByEncoding(ground_plane_actor, {0.3, 0.4, 0.1}, usePureColor);

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

vtkSmartPointer<vtkActor> createColoredMeshActor(const shape_msgs::msg::Mesh &mesh, const std::array<double, 4> &color_rgba, bool visible) {

	auto actor = createActorFromMesh(mesh);
	actor->GetProperty()->SetColor(color_rgba[0], color_rgba[1], color_rgba[2]);
	if (color_rgba[3] < 1.0) {
		actor->GetProperty()->SetOpacity(color_rgba[3]);
	}
	return actor;

}

std::vector<vtkSmartPointer<vtkActor>> createColoredMeshActors(const std::vector<shape_msgs::msg::Mesh> &meshes,
															   const std::array<double, 4> &color_rgba,
															   bool visible) {

	std::vector<vtkSmartPointer<vtkActor>> actors;

	for (const auto& mesh : meshes) {
		actors.push_back(createColoredMeshActor(mesh, color_rgba, visible));
	}

	return actors;

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

void VtkPointCloudVisualization::updatePoints(const std::vector<mgodpl::math::Vec3d> &points) {

	assert(!points.empty());

	vtkNew<vtkPoints> pointsVtk;
	vtkNew<vtkCellArray> cells;

	for (const auto &p: points) {
		cells->InsertNextCell(1);
		cells->InsertCellPoint(pointsVtk->InsertNextPoint(p.components.data()));
	}

	visitOrderVisualizationData->SetPoints(pointsVtk);
	visitOrderVisualizationData->SetVerts(cells);

	visitOrderVisualizationData->Modified();

}
