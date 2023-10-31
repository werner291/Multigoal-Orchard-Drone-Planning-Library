// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/31/23.
//

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
#include "convex_hull.h"

// Use CGAL (EPIC) to compute the convex hull of a set of points.

mgodpl::Mesh mgodpl::visibility::convexHull(const std::vector<math::Vec3d> &points) {

		// Step one: convert the points to a CGAL representation.

		// The kernel we use for CGAL (EPIC)
		using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;

		std::vector<Kernel::Point_3> cgalPoints = points | ranges::views::transform([](const math::Vec3d &point) {
			return Kernel::Point_3(point.x(), point.y(), point.z());
		}) | ranges::to<std::vector>();

		// Step 2: get the convex hull of the points.
		CGAL::Polyhedron_3<Kernel> polyhedron;

		// Compute the convex hull.
		CGAL::convex_hull_3(cgalPoints.begin(), cgalPoints.end(), polyhedron);

		// We keep only the vertices associated with those triangles.
		Mesh mesh;

		std::unordered_map<Kernel::Point_3, int> index_translation;

		// Iterate over all triangles in the chull.
		for (auto face = polyhedron.facets_begin(); face != polyhedron.facets_end(); ++face) {

			// Get the normal of the triangle.
			auto normal = face->plane().orthogonal_vector();

			// If the normal faces away from the eye, we keep the triangle.
			if (normal * (face->halfedge()->vertex()->point() - Kernel::Point_3(eye.x(), eye.y(), eye.z())) > 0) {

				math::Vec3i triangle(-1,-1,-1);
				int triangle_vertex_i = 0;

				auto vertex = face->facet_begin();

				// Iterate over the vertices of the triangle.
				do  {

					// Get the index of the vertex.
					auto index = index_translation.find(vertex->vertex()->point());

					if (index == index_translation.end()) {
						// If the vertex is not yet in the translation table, add it.
						index = index_translation.insert({vertex->vertex()->point(), index_translation.size()}).first;
					}

					// Add the index to the triangle.
					triangle[triangle_vertex_i++] = index->second;

					assert(triangle_vertex_i <= 3);

				} while (++vertex != face->facet_begin());

				// Add the triangle to the mesh.
				mesh.faces.push_back(triangle);
			}
		}

		return {
			eye, mesh
		};


}
