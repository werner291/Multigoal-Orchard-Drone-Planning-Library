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
	const size_t SUBDIVISIONS = 30;

	AABBGrid grid_coords(
			AABBd(Vec3d(-3.0, -3.0, 0.0), Vec3d(3.0, 3.0, 6.0)),
			SUBDIVISIONS,SUBDIVISIONS,SUBDIVISIONS);


	Vec3d view_center1 {2.0, 2.55, 2.1};
	Vec3d view_center2 {0.5, 2.55, 2.1};

	// And then render it in VTK as a set of cubes...

	vtkNew<vtkNamedColors> colors;



	vtkNew<vtkPolyData> polydata;

	// Create anything you want here, we will use a cube for the demo.
	vtkNew<vtkCubeSource> cubeSource;

	vtkNew<vtkGlyph3D> glyph3D;
	glyph3D->SetSourceConnection(cubeSource->GetOutputPort());
	glyph3D->SetInputData(polydata);
	glyph3D->SetScaleFactor(6.0 / (double) SUBDIVISIONS);
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
	sphereSource->SetCenter(view_center1.x(), view_center1.y(), view_center1.z());
	sphereSource->SetRadius(0.1);
	vtkNew<vtkPolyDataMapper> sphereMapper;
	sphereMapper->SetInputConnection(sphereSource->GetOutputPort());
	vtkNew<vtkActor> sphereActor;
	sphereActor->SetMapper(sphereMapper);
	sphereActor->GetProperty()->SetColor(colors->GetColor3d("Red").GetData());

	SimpleVtkViewer viewer;
	viewer.addActor(actor);
	viewer.addActor(sphereActor);

	viewer.addMesh(treeMeshes.leaves_mesh, Vec3d(0.0, 0.5, 0.0));
	viewer.addMesh(treeMeshes.trunk_mesh, Vec3d(0.5, 0.3, 0.1));

	double t = 0;
	double step = 0.01;

	viewer.addTimerCallback([&]() {

		Vec3d view_center = view_center1 * (1.0 - t) + view_center2 * t;

		Grid3D<bool> grid(SUBDIVISIONS,SUBDIVISIONS,SUBDIVISIONS, false);
		vtkNew<vtkPoints> points = grid_to_points(SUBDIVISIONS, grid_coords, grid, false);

		// For every leaf in the tree, set the corresponding grid cell to true.
		for (const auto &triangle: treeMeshes.leaves_mesh.triangles) {

			Vec3d a = toVec3d(treeMeshes.leaves_mesh.vertices[triangle.vertex_indices[0]]);
			Vec3d b = toVec3d(treeMeshes.leaves_mesh.vertices[triangle.vertex_indices[1]]);
			Vec3d c = toVec3d(treeMeshes.leaves_mesh.vertices[triangle.vertex_indices[2]]);

			Triangle tri {
					a,b,c
			};

			mgodpl::voxel_visibility::cast_occlusion(grid_coords, grid, tri, view_center);

//			break;
		}

		polydata->SetPoints(grid_to_points(SUBDIVISIONS, grid_coords, grid, false));

		sphereSource->SetCenter(view_center.x(), view_center.y(), view_center.z());
		t += step;

	});

	viewer.start();

}