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
#include <vtkCamera.h>
#include <vtkRenderer.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>

#include "../visualization/SimpleVtkViewer.h"
#include "../math/AABBGrid.h"
#include "../math/grid_utils.h"
#include "../visibility/GridVec.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../visibility/voxel_visibility.h"
#include "../math/Triangle.h"
#include "../visibility/octree_visibility.h"

#include <vtkSphereSource.h>
#include <boost/range/irange.hpp>

using namespace std;
using namespace mgodpl;
using namespace math;
using namespace tree_meshes;
using namespace visibility;
using namespace voxel_visibility;

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

using namespace visibility;

//std::pair<vtkSmartPointer<vtkPoints>,vtkSmartPointer<vtkDoubleArray>> octree_to_points(const AABBd& parent, const VisibilityOctree& octree) {
//	vtkNew<vtkPoints> points;
//	vtkNew<vtkDoubleArray> values;
//
//	std::vector<std::pair<AABBd, const VisibilityOctree::Node*>> stack {
//		{parent, &octree._root}
//	};
//
//	while (!stack.empty()) {
//		auto [aabb, node] = stack.back();
//		stack.pop_back();
//
//		if (const auto& leaf = std::get_if<VisibilityOctree::LeafNode>(node)) {
//			if (leaf->data) {
//				points->InsertNextPoint(aabb.center().x(), aabb.center().y(), aabb.center().z());
//				values->InsertNextValue(aabb.size().x());
//			}
//		} else if (const auto& split = std::get_if<VisibilityOctree::SplitNode>(node)) {
//			for (size_t i = 0; i < 8; ++i) {
//				stack.emplace_back(childAABB(aabb, i), &split->children->at(i));
//			}
//		}
//	}
//
//	return {
//		points,
//		values
//	};
//
//}

Vec3d toVec3d(const geometry_msgs::msg::Point& p) {
	return {p.x, p.y, p.z};
}

// A trait for
int main(int argc, char **argv) {

	bool record = false;
	for (int i = 0; i < argc; ++i) {
		if (strcmp(argv[i], "--record") == 0) {
			record = true;
		}
	}

	auto treeMeshes = loadTreeMeshes("appletree");

	// Let's create a nxnxn grid of boolean values...
	const size_t SUBDIVISIONS = 30;

	AABBGrid grid_coords(
			AABBd(Vec3d(-3.0, -3.0, 0.0), Vec3d(3.0, 3.0, 6.0)),
			SUBDIVISIONS,SUBDIVISIONS,SUBDIVISIONS);

	Vec3d view_center1 {2.0, 2.55, 2.1};
	Vec3d view_center2 {-2.0, 2.55, 2.1};

	// And then render it in VTK as a set of cubes...

	vtkNew<vtkNamedColors> colors;
	vtkNew<vtkPolyData> polydata;

	// Create anything you want here, we will use a cube for the demo.
	vtkNew<vtkCubeSource> cubeSource;

	vtkNew<vtkGlyph3D> glyph3D;
	glyph3D->SetSourceConnection(cubeSource->GetOutputPort());
	glyph3D->SetInputData(polydata);
//	glyph3D->SetScaleFactor(6.0 / (double) SUBDIVISIONS);
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
	double step = 0.1;

	Grid3D<bool> seen_space(SUBDIVISIONS,SUBDIVISIONS,SUBDIVISIONS, false);

	Vec3d tree_center(0.0,0.0,2.0);

	viewer.viewerRenderer->GetActiveCamera()->SetPosition(-2.0,10.0,8.0);
	viewer.viewerRenderer->GetActiveCamera()->SetFocalPoint(0.0, 0.0, 2.0);
	viewer.viewerRenderer->GetActiveCamera()->SetViewUp(0.0, 0.0, 1.0);

	enum VisualizedVolume {
		OCCLUDED,
		UNSEEN
	};

	VisualizedVolume visualized_volume = OCCLUDED;

	std::vector<Triangle> triangles;

	// For every leaf in the tree, set the corresponding grid cell to true.
	for (const auto &triangle: treeMeshes.leaves_mesh.triangles) {

		Vec3d a = toVec3d(treeMeshes.leaves_mesh.vertices[triangle.vertex_indices[0]]);
		Vec3d b = toVec3d(treeMeshes.leaves_mesh.vertices[triangle.vertex_indices[1]]);
		Vec3d c = toVec3d(treeMeshes.leaves_mesh.vertices[triangle.vertex_indices[2]]);

		triangles.emplace_back(a,b,c);
	}

	viewer.addTimerCallback([&]() {

		Vec3d view_center(
				std::cos(t) * 3.0,
				std::sin(t) * 3.0,
				2.0
				);

		const Grid3D<bool>& occluded_space = cast_occlusion(grid_coords, triangles, view_center);
//		const VisibilityOctree& occluded_space_octree = cast_occlusion_batch_sorting(grid_coords.baseAABB(),
//																					 triangles,
//																					 view_center);

//		// All non-occluded space will now be added to the seen space.
//		for (const auto &coord: boost::irange(0, (int) SUBDIVISIONS)) {
//			for (const auto &coord2: boost::irange(0, (int) SUBDIVISIONS)) {
//				for (const auto &coord3: boost::irange(0, (int) SUBDIVISIONS)) {
//					if (!occluded_space[{coord, coord2, coord3}]) {
//						seen_space[{coord, coord2, coord3}] = true;
//					}
//				}
//			}
//		}

		polydata->SetPoints(grid_to_points(SUBDIVISIONS, grid_coords, occluded_space, false));
//		polydata->SetPoints(grid_to_points(SUBDIVISIONS, grid_coords, seen_space, true));

//		const auto& [points, values] = octree_to_points(grid_coords.baseAABB(), occluded_space_octree);
//		polydata->SetPoints(points);
//		polydata->GetPointData()->SetScalars(values);

		sphereSource->SetCenter(view_center.x(), view_center.y(), view_center.z());
		t += step;

		if (t > 2.0 * M_PI) {
			viewer.stop();
			t = 0;
		} else {
			std::cout << "T: " << t << " / " << 2.0 * M_PI << std::endl;
		}

	});

	if (record)
		viewer.startRecording("cubeviz_all.ogv");

	viewer.start();

}