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
#include "../utilities/math/AABBGrid.h"
#include "../utilities/math/vecmath_utils.h"
#include "../utilities/math/grid_utils.h"
#include "../utilities/GridVec.h"
#include "../utilities/msgs_utilities.h"
#include "../voxel_visibility.h"
#include <boost/range/irange.hpp>
#include <vtkSphereSource.h>

using namespace std;

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

int main(int argc, char **argv) {

	auto treeMeshes = loadTreeMeshes("appletree");

	// Let's create a nxnxn grid of boolean values...
	const size_t SUBDIVISIONS = 30;

	mgodpl::math::AABBGrid grid_coords(
			Eigen::AlignedBox3d(Eigen::Vector3d(-3.0, -3.0, 0.0), Eigen::Vector3d(3.0, 3.0, 6.0)),
			SUBDIVISIONS,SUBDIVISIONS,SUBDIVISIONS);

	Grid3D<bool> grid(SUBDIVISIONS,SUBDIVISIONS,SUBDIVISIONS, false);

	// For every leaf in the tree, set the corresponding grid cell to true.
	for (const auto &triangle: treeMeshes.leaves_mesh.triangles) {

		Eigen::Vector3d a = toEigen(treeMeshes.leaves_mesh.vertices[triangle.vertex_indices[0]]);
		Eigen::Vector3d b = toEigen(treeMeshes.leaves_mesh.vertices[triangle.vertex_indices[1]]);
		Eigen::Vector3d c = toEigen(treeMeshes.leaves_mesh.vertices[triangle.vertex_indices[2]]);

		Eigen::AlignedBox3d triangle_aabb(a, a);
		triangle_aabb.extend(b);
		triangle_aabb.extend(c);

		auto grid_aabb = *grid_coords.touchedCoordinates(triangle_aabb);

		for (int x : boost::irange(grid_aabb.min().x(), grid_aabb.max().x() + 1)) {
			for (int y : boost::irange(grid_aabb.min().y(), grid_aabb.max().y() + 1)) {
				for (int z : boost::irange(grid_aabb.min().z(), grid_aabb.max().z() + 1)) {
					grid[{(size_t)x, (size_t)y, (size_t)z}] = true;
				}
			}
		}

	}

	Eigen::Vector3d view_center {1.2, 2.55, 2.1};

	auto visible = mgodpl::voxel_visibility::opaque_to_visible(grid_coords, grid, view_center);

	assert(visible[grid_coords.getGridCoordinates(view_center - Eigen::Vector3d::UnitX()).value()]);
	assert(visible[grid_coords.getGridCoordinates(view_center + Eigen::Vector3d::UnitX()).value()]);

	// And then render it in VTK as a set of cubes...

	vtkNew<vtkNamedColors> colors;

	vtkNew<vtkPoints> points = grid_to_points(SUBDIVISIONS, grid_coords, visible, false);

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

	// And add a sphere for the viewpoint.
	vtkNew<vtkSphereSource> sphereSource;
	sphereSource->SetCenter(view_center.x(), view_center.y(), view_center.z());
	sphereSource->SetRadius(0.1);

	vtkNew<vtkPolyDataMapper> sphereMapper;
	sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

	vtkNew<vtkActor> sphereActor;
	sphereActor->SetMapper(sphereMapper);
	sphereActor->GetProperty()->SetColor(colors->GetColor3d("Red").GetData());

	SimpleVtkViewer viewer;
	viewer.addActor(actor);
	viewer.addActor(sphereActor);

	addTreeMeshesToViewer(viewer, treeMeshes);

	viewer.start();

}