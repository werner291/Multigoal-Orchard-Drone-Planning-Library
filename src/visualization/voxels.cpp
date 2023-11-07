#include <vtkPolyData.h>
#include <vtkCubeSource.h>
#include <vtkGlyph3D.h>
#include <vtkPolyDataMapper.h>
#include "voxels.h"

#include "../math/AABBGrid.h"
#include "../math/grid_utils.h"
#include "../visibility/GridVec.h"
#include "../math/Vec3.h"

vtkNew <vtkPoints> mgodpl::visualization::grid_to_points(const size_t SUBDIVISIONS,
														 const mgodpl::math::AABBGrid &grid_coords,
														 const Grid3D<bool> &grid,
														 bool negate) {
	vtkNew <vtkPoints> points;

	// Allocate a point for every true value in the grid.
	for (size_t x = 0; x < SUBDIVISIONS; x++) {
		for (size_t y = 0; y < SUBDIVISIONS; y++) {
			for (size_t z = 0; z < SUBDIVISIONS; z++) {

				math::Vec3i coords = {(int)x, (int)y, (int)z};

				bool invisible = grid[coords];

				if (negate) {
					invisible = !invisible;
				}

				bool neighbour_visible = grid.voxel_has_different_neighbor(coords);

				if (invisible && neighbour_visible) {

					auto aabb = grid_coords.getAABB(coords);

					points->InsertNextPoint(aabb->center().x(), aabb->center().y(), aabb->center().z());
				}
			}
		}
	}
	return points;
}

mgodpl::visualization::VtkVoxelGrid::VtkVoxelGrid(const mgodpl::math::AABBGrid &grid_coords,
												  const mgodpl::Grid3D<bool> &grid,
												  const mgodpl::math::Vec3d &rgb,
												  bool negate) : grid_coords(grid_coords), negate(negate) {

	vtkNew<vtkNamedColors> colors;

	// Create anything you want here, we will use a cube for the demo.
	vtkNew<vtkCubeSource> cubeSource;

	math::Vec3d cell_size = grid_coords.cellSize();
	assert(cell_size.x() == cell_size.y() && cell_size.y() == cell_size.z());

	vtkNew<vtkGlyph3D> glyph3D;
	glyph3D->SetSourceConnection(cubeSource->GetOutputPort());
	glyph3D->SetInputData(polydata);
	glyph3D->SetScaleFactor(cell_size.x());
	glyph3D->Update();

	vtkNew<vtkPolyDataMapper> mapper;
	mapper->SetInputConnection(glyph3D->GetOutputPort());

	actor->SetMapper(mapper);

	update(grid);

}

void mgodpl::visualization::VtkVoxelGrid::update(const mgodpl::Grid3D<bool> &grid) {
	polydata->SetPoints(grid_to_points(grid_coords.size().x(), grid_coords, grid, negate));
}
