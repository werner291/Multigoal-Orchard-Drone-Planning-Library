// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/18/24.
//

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_data_structure_3.h>
#include <CGAL/Delaunay_triangulation_3.h>

#include "benchmark_function_macros.h"
#include "../experiment_utils/default_colors.h"
#include "../experiment_utils/LoadedTreeModel.h"
#include "../visualization/visualization_function_macros.h"
#include "../visualization/VtkTriangleSetVisualization.h"
#include "../math/Vec3.h"

REGISTER_BENCHMARK(tree_complexity_metrics) {
	auto names = mgodpl::tree_meshes::getTreeModelNames();

	for (const auto &name: names) {
		// Load a tree model:
		auto tree = mgodpl::tree_meshes::loadTreeMeshes(name).trunk_mesh;

		// Compute the hierarchical alpha shapes using CGAL:
		using Gt = CGAL::Exact_predicates_inexact_constructions_kernel;
		using Triangulation_3 = CGAL::Delaunay_triangulation_3<Gt>;
		using Point = Gt::Point_3;

		std::vector<Point> points;
		for (const auto &vertex: tree.vertices) {
			points.push_back(Point(vertex[0], vertex[1], vertex[2]));
		}

		Triangulation_3 dt(points.begin(), points.end());

		// Now, iterate over all the cells and output both the volume of the cell and the size of the sphere:
		for (auto cell = dt.finite_cells_begin(); cell != dt.finite_cells_end(); ++cell) {
			// First, grab the tetrahedron:
			auto tetrahedron = dt.tetrahedron(cell);

			// Circumcenter:
			auto circumcenter = CGAL::circumcenter(tetrahedron);
			double radius = CGAL::squared_distance(circumcenter, tetrahedron[0]);

			auto volume = tetrahedron.volume();

			results[name]["cell_volume"].append(volume);
			results[name]["cell_radius"].append(std::sqrt(radius));
		}
	}
}

using namespace mgodpl;

REGISTER_VISUALIZATION(tree_complexity_metrics_viz) {
	auto tree_model = mgodpl::tree_meshes::loadTreeMeshes("appletree").trunk_mesh;

	viewer.addMesh(tree_model, mgodpl::WOOD_COLOR);

	VtkTriangleSetVisualization vis(1, 0, 1, 0.5);
	viewer.addActor(vis.getActor());

	// Compute the hierarchical alpha shapes using CGAL:
	using Gt = CGAL::Exact_predicates_inexact_constructions_kernel;
	using Triangulation_3 = CGAL::Delaunay_triangulation_3<Gt>;
	using Point = Gt::Point_3;

	std::vector<Point> points;
	for (const auto &vertex: tree_model.vertices) {
		points.push_back(Point(vertex[0], vertex[1], vertex[2]));
	}

	Triangulation_3 dt(points.begin(), points.end());

	std::vector<CGAL::Tetrahedron_3<Gt> > tetrahedra;
	for (auto cell = dt.finite_cells_begin(); cell != dt.finite_cells_end(); ++cell) {
		auto tetrahedron = dt.tetrahedron(cell);
		tetrahedra.push_back(tetrahedron);
	}
	// Sort by circumradius:
	std::sort(tetrahedra.begin(),
	          tetrahedra.end(),
	          [](const auto &a, const auto &b) {
		          auto circumcenter_a = CGAL::circumcenter(a);
		          auto circumcenter_b = CGAL::circumcenter(b);

		          return CGAL::squared_distance(circumcenter_a, a[0]) < CGAL::squared_distance(circumcenter_b, b[0]);
	          });

	std::vector<std::array<math::Vec3d, 3> > triangles;

	auto it = tetrahedra.begin();

	viewer.addTimerCallback([&]() {
		for (int i = 0; i < 1000 && it != tetrahedra.end(); ++i) {
			// First, grab the tetrahedron:
			auto tetrahedron = *it;

			math::Vec3d p1(tetrahedron[0][0], tetrahedron[0][1], tetrahedron[0][2]);
			math::Vec3d p2(tetrahedron[1][0], tetrahedron[1][1], tetrahedron[1][2]);
			math::Vec3d p3(tetrahedron[2][0], tetrahedron[2][1], tetrahedron[2][2]);
			math::Vec3d p4(tetrahedron[3][0], tetrahedron[3][1], tetrahedron[3][2]);

			triangles.push_back({p1, p2, p3});
			triangles.push_back({p1, p2, p4});
			triangles.push_back({p1, p3, p4});
			triangles.push_back({p2, p3, p4});

			vis.updateTriangles(triangles);

			std::cout << "Shown " << triangles.size() << " triangles" << std::endl;

			++it;
		}
	});

	viewer.start();
}
