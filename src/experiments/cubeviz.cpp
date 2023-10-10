// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <vector>

#include <vtkActor.h>
#include <vtkCubeSource.h>
#include <vtkGlyph3D.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <boost/range/irange.hpp>

#include "../visualization/SimpleVtkViewer.h"
#include "../utilities/math/AABBGrid.h"
#include "../utilities/GridVec.h"
#include "../utilities/msgs_utilities.h"

using namespace std;

/**
 * A struct that produces a discretely space-filling spiral centered on (0,0,0),
 * returning points in order of increasing distance from the origin.
 *
 * In 1D, that'd be the sequence:
 * 	0, 1, -1, 2, -2, 3, -3, 4, -4, ...
 *
 * In 2D, that'd be the sequence:
 *  (0,0),
 *  (1,0), (0,1), (-1,0), (0,-1),
 *  (1,1), (-1,1), (-1,-1), (1,-1),
 *  (2,0), (
 */
struct SpiralCoordinates {



};

Eigen::Vector3i
direction_to_faced_neighbor(const Eigen::Vector3i &direction) {// Find the axis with biggest absolute value.
	Eigen::Vector3i step;

	if (std::abs(direction.x()) >= std::abs(direction.y()) && std::abs(direction.x()) >= std::abs(direction.z())) {
		step = {direction.x() > 0 ? 1 : -1, 0, 0};
	} else if (std::abs(direction.y()) >= std::abs(direction.x()) && std::abs(direction.y()) >= std::abs(direction.z())) {
		step = {0, direction.y() > 0 ? 1 : -1, 0};
	} else {
		step = {0, 0, direction.z() > 0 ? 1 : -1};
	}
	return step;
}

Grid3D<bool> compute_visible(const Grid3D<bool>& occluding, const std::array<size_t,3>& view_center) {

	auto sizes = occluding.size();

	Grid3D<bool> visible(sizes[0], sizes[1], sizes[2], false);

	visible[view_center] = true;

	// Generate all coordinates in the grid. (Ugh, can we generate these with a sequence/spiral of some sort instead?)

	std::vector<Eigen::Vector3i> coordinates;

	for (int x : boost::irange(0, (int)sizes[0])) {
		for (int y : boost::irange(0, (int)sizes[1])) {
			for (int z : boost::irange(0, (int)sizes[2])) {
				coordinates.push_back({x, y, z});
			}
		}
	}

	// Sort by distance from the view center.

	std::sort(coordinates.begin(), coordinates.end(), [&view_center](const auto& a, const auto& b) {
		return (a[0]-view_center[0])*(a[0]-view_center[0]) + (a[1]-view_center[1])*(a[1]-view_center[1]) + (a[2]-view_center[2])*(a[2]-view_center[2]) <
				(b[0]-view_center[0])*(b[0]-view_center[0]) + (b[1]-view_center[1])*(b[1]-view_center[1]) + (b[2]-view_center[2])*(b[2]-view_center[2]);
	});

	// Iterate in order:

	for (Eigen::Vector3i cell_pt : coordinates) {

		if (cell_pt == Eigen::Vector3i((int)view_center[0], (int)view_center[1], (int)view_center[2])) {
			continue;
		}

		// Find the neighboring (shared face!) cell that's closest to the view center.

		// Direction to the eye center.
		Eigen::Vector3i direction =  Eigen::Vector3i((int)view_center[0], (int)view_center[1], (int)view_center[2]) - cell_pt;
		Eigen::Vector3i step = direction_to_faced_neighbor(direction);

//		std::cout << "Eye center: " << view_center[0] << ", " << view_center[1] << ", " << view_center[2] << std::endl;
//		std::cout << "Cell: " << cell_pt.transpose() << std::endl;
//		std::cout << "Direction: " << direction.transpose() << std::endl;
//		std::cout << "Step: " << step.transpose() << std::endl;

		// Assert in-bounds.
		assert(cell_pt.x()+step.x() >= 0); assert(cell_pt.x()+step.x() < sizes[0]);
		assert(cell_pt.y()+step.y() >= 0); assert(cell_pt.y()+step.y() < sizes[1]);
		assert(cell_pt.z()+step.z() >= 0); assert(cell_pt.z()+step.z() < sizes[2]);

		// Find the neighboring cell.
		bool neighbour_visible = visible[{
				(size_t) (cell_pt.x()+step.x()),
				(size_t) (cell_pt.y()+step.y()),
				(size_t) (cell_pt.z()+step.z())
		}];

		bool self_is_occluded = occluding[{(size_t)cell_pt.x(), (size_t)cell_pt.y(), (size_t)cell_pt.z()}];

//		// If the neighbor is visible, this cell is visible too.
//		if (neighbour_visible && !self_is_occluded) {
//			visible[{x,y,z}] = true;
//		}

		visible[{(size_t)cell_pt.x(), (size_t)cell_pt.y(), (size_t)cell_pt.z()}] = !self_is_occluded && neighbour_visible;
	}

	return visible;

}

int main(int argc, char **argv) {

	auto treeMeshes = loadTreeMeshes("appletree");

	// Let's create a nxnxn grid of boolean values...

	const size_t SUBDIVISIONS = 100;

	mgodpl::math::AABBGrid grid_coords(
			Eigen::AlignedBox3d(Eigen::Vector3d(-5.0, -5.0, 0.0), Eigen::Vector3d(5.0, 5.0, 10.0)),
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

	auto visible = compute_visible(grid, {1,4,4});

	// And then render it in VTK as a set of cubes...

	vtkNew<vtkNamedColors> colors;

	vtkNew<vtkPoints> points;

	// Allocate a point for every true value in the grid.
	for (int x = 0; x < SUBDIVISIONS; x++) {
		for (int y = 0; y < SUBDIVISIONS; y++) {
			for (int z = 0; z < SUBDIVISIONS; z++) {

				bool invisible = !visible[{(size_t)x, (size_t)y, (size_t)z}];

				bool neighbour_visible = (x == 0 || visible[{(size_t) (x-1), (size_t) y, (size_t) z}]) ||
										 (x == SUBDIVISIONS-1 || visible[{(size_t) (x+1), (size_t) y, (size_t) z}]) ||
										 (y == 0 || visible[{(size_t) x, (size_t) (y-1), (size_t) z}]) ||
										 (y == SUBDIVISIONS-1 || visible[{(size_t) x, (size_t) (y+1), (size_t) z}]) ||
										 (z == 0 || visible[{(size_t) x, (size_t) y, (size_t) (z-1)}]) ||
										 (z == SUBDIVISIONS-1 || visible[{(size_t) x, (size_t) y, (size_t) (z+1)}]);

				if (invisible && neighbour_visible) {

					auto aabb = grid_coords.getAABB({(size_t)x, (size_t)y, (size_t)z});

					points->InsertNextPoint(aabb->center().x(), aabb->center().y(), aabb->center().z());
				}
			}
		}
	}

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

	SimpleVtkViewer viewer;
	viewer.addActor(actor);


	addTreeMeshesToViewer(viewer, treeMeshes);

	viewer.start();

}