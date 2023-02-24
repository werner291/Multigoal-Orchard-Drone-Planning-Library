// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 24-2-23.
//

#include "alpha_shape.h"
#include <CGAL/Fixed_alpha_shape_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_3.h>
#include <CGAL/Triangulation_vertex_base_3.h>
#include <CGAL/Regular_triangulation_3.h>
#include <CGAL/Triangulation_cell_base_with_info_3.h>
#include <CGAL/algorithm.h>
#include <CGAL/Polyhedron_3.h>
#include <iostream>
#include <vector>
#include <CGAL/Fixed_alpha_shape_vertex_base_3.h>
#include <CGAL/Fixed_alpha_shape_cell_base_3.h>
#include <range/v3/all.hpp>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Regular_triangulation_vertex_base_3<K> Vbb;
typedef CGAL::Fixed_alpha_shape_vertex_base_3<K, Vbb> Vb;
typedef CGAL::Regular_triangulation_cell_base_3<K> Rcb;
typedef CGAL::Fixed_alpha_shape_cell_base_3<K, Rcb> Cb;
typedef CGAL::Triangulation_data_structure_3<Vb, Cb> Tds;
typedef CGAL::Regular_triangulation_3<K, Tds> Triangulation_3;
typedef CGAL::Fixed_alpha_shape_3<Triangulation_3> Fixed_alpha_shape_3;
typedef Fixed_alpha_shape_3::Cell_handle Cell_handle;
typedef Fixed_alpha_shape_3::Vertex_handle Vertex_handle;
typedef Fixed_alpha_shape_3::Facet Facet;
typedef Fixed_alpha_shape_3::Edge Edge;
typedef Triangulation_3::Weighted_point Weighted_point;
typedef Triangulation_3::Bare_point Bare_point;

shape_msgs::msg::Mesh alphaShape(const std::vector<Eigen::Vector3d> &points, double alpha) {

	// Construct the alpha shape
	std::vector<Weighted_point> lwp = points | ranges::views::transform([](const Eigen::Vector3d &p) {
		return Weighted_point(Bare_point(p.x(), p.y(), p.z()), 1);
	}) | ranges::to_vector;
	Fixed_alpha_shape_3 as(lwp.begin(), lwp.end(), alpha);

	// Extract the alpha shape
	std::vector<Facet> facets;
	as.get_alpha_shape_facets(std::back_inserter(facets), Fixed_alpha_shape_3::REGULAR);

	// Convert the alpha shape to a mesh
	shape_msgs::msg::Mesh mesh;

	std::unordered_map<Vertex_handle, uint32_t> vertex_map;

	for (const Facet &facet: facets) {
		shape_msgs::msg::MeshTriangle triangle;

		// Find the vertices of the facet
		for (int i = 1; i <= 3; i++) {
			Vertex_handle vertex = facet.first->vertex((facet.second + i) % 4);
			auto it = vertex_map.find(vertex);
			if (it == vertex_map.end()) {
				// Add the vertex to the mesh
				geometry_msgs::msg::Point point;
				point.x = vertex->point().x();
				point.y = vertex->point().y();
				point.z = vertex->point().z();
				mesh.vertices.push_back(point);

				// Add the vertex to the map
				vertex_map[vertex] = mesh.vertices.size() - 1;
			}

			// Add the vertex to the triangle
			triangle.vertex_indices[i - 1] = vertex_map[vertex];
		}

		mesh.triangles.push_back(triangle);
	}

	return mesh;

}