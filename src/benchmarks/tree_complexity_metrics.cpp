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
#include "../experiment_utils/LoadedTreeModel.h"
#include "../visualization/visualization_function_macros.h"

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
