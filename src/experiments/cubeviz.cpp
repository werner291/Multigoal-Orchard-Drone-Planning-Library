// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <vtkActor.h>
#include <vtkCubeSource.h>
#include <vtkGlyph3D.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>

#include "../visualization/SimpleVtkViewer.h"
#include "../math/AABBGrid.h"
#include "../math/grid_utils.h"
#include "../experiment_utils/GridVec.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/voxel_visibility.h"
#include "../math/Triangle.h"

#include <vtkSphereSource.h>
#include <boost/range/irange.hpp>

using namespace std;
using namespace mgodpl;
using namespace math;
using namespace tree_meshes;

vtkNew<vtkPoints>
grid_to_points(const size_t SUBDIVISIONS,
			   const mgodpl::math::AABBGrid &grid_coords,
			   const Grid3D<bool> &grid,
			   bool negate) {
	vtkNew<vtkPoints> points;

	// Allocate a point for every true value in the grid.
	for (int x = 0; x < SUBDIVISIONS; x++) {
		for (int y = 0; y < SUBDIVISIONS; y++) {
			for (int z = 0; z < SUBDIVISIONS; z++) {

				bool invisible = grid[{x, y, z}];
				
				if (negate) {
					invisible = !invisible;
				}
				
				bool neighbour_visible = grid.voxel_has_different_neighbor({x, y, z});

				if (invisible && neighbour_visible) {

					auto aabb = grid_coords.getAABB({x, y, z});

					points->InsertNextPoint(aabb->center().x(), aabb->center().y(), aabb->center().z());
				}
			}
		}
	}
	return points;
}

Vec3d toVec3d(const geometry_msgs::msg::Point& p) {
	return Vec3d(p.x, p.y, p.z);
}

int main(int argc, char **argv) {

	auto treeMeshes = loadTreeMeshes("appletree");

	// Let's create a nxnxn grid of boolean values...
	const size_t SUBDIVISIONS = 100;

	AABBGrid grid_coords(
			AABBd(Vec3d(-3.0, -3.0, 0.0), Vec3d(3.0, 3.0, 6.0)),
			SUBDIVISIONS,SUBDIVISIONS,SUBDIVISIONS);

	Grid3D<bool> grid(SUBDIVISIONS,SUBDIVISIONS,SUBDIVISIONS, false);

	Vec3d view_center {1.2, 2.55, 2.1};

	std::optional<Vec3d> triangle_center;

	// For every leaf in the tree, set the corresponding grid cell to true.
	for (const auto &triangle: treeMeshes.leaves_mesh.triangles) {

		Vec3d a = toVec3d(treeMeshes.leaves_mesh.vertices[triangle.vertex_indices[0]]);
		Vec3d b = toVec3d(treeMeshes.leaves_mesh.vertices[triangle.vertex_indices[1]]);
		Vec3d c = toVec3d(treeMeshes.leaves_mesh.vertices[triangle.vertex_indices[2]]);

		Triangle tri {
			a,b,c
		};

		mgodpl::voxel_visibility::cast_occlusion(grid_coords, grid, tri, view_center);

		if (!triangle_center.has_value()) {
			triangle_center = (a + b + c) / 3.0;
		}
		break;

	}



//	auto visible = mgodpl::voxel_visibility::opaque_to_visible(grid, view_center, false);

//	assert(visible[grid_coords.getGridCoordinates(view_center - Vec3d::UnitX()).value()]);
//	assert(visible[grid_coords.getGridCoordinates(view_center + Vec3d::UnitX()).value()]);

	// And then render it in VTK as a set of cubes...

	vtkNew<vtkNamedColors> colors;

	vtkNew<vtkPoints> points = grid_to_points(SUBDIVISIONS, grid_coords, grid, false);

	vtkNew<vtkPolyData> polydata;
	polydata->SetPoints(points);

	// Create anything you want here, we will use a cube for the demo.
	vtkNew<vtkCubeSource> cubeSource;

	vtkNew<vtkGlyph3D> glyph3D;
	glyph3D->SetSourceConnection(cubeSource->GetOutputPort());
	glyph3D->SetInputData(polydata);
	glyph3D->SetScaleFactor(10.0 / (double) SUBDIVISIONS);
	glyph3D->Update();

	// Visualize
	vtkNew<vtkPolyDataMapper> mapper;
	mapper->SetInputConnection(glyph3D->GetOutputPort());

	vtkNew<vtkActor> actor;
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(colors->GetColor3d("Salmon").GetData());
	actor->GetProperty()->SetOpacity(0.5);

	// And add a sphere for the viewpoint.
	vtkNew<vtkSphereSource> sphereSource;
	sphereSource->SetCenter(view_center.x(), view_center.y(), view_center.z());
	sphereSource->SetRadius(0.1);
	vtkNew<vtkPolyDataMapper> sphereMapper;
	sphereMapper->SetInputConnection(sphereSource->GetOutputPort());
	vtkNew<vtkActor> sphereActor;
	sphereActor->SetMapper(sphereMapper);
	sphereActor->GetProperty()->SetColor(colors->GetColor3d("Red").GetData());

	// And add a sphere for the triangle center.
	vtkNew<vtkSphereSource> triangleSphereSource;
	triangleSphereSource->SetCenter(triangle_center->x(), triangle_center->y(), triangle_center->z());
	triangleSphereSource->SetRadius(0.2);
	vtkNew<vtkPolyDataMapper> triangleSphereMapper;
	triangleSphereMapper->SetInputConnection(triangleSphereSource->GetOutputPort());
	vtkNew<vtkActor> triangleSphereActor;
	triangleSphereActor->SetMapper(triangleSphereMapper);
	triangleSphereActor->GetProperty()->SetColor(colors->GetColor3d("Blue").GetData());

	SimpleVtkViewer viewer;
	viewer.addActor(actor);
	viewer.addActor(sphereActor);
	viewer.addActor(triangleSphereActor);

	viewer.addMesh(treeMeshes.leaves_mesh, Vec3d(0.0, 0.5, 0.0));
	viewer.addMesh(treeMeshes.trunk_mesh, Vec3d(0.5, 0.3, 0.1));

	viewer.start();

}