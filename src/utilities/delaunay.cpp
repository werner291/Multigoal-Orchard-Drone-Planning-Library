// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-3-23.
//

#include <shape_msgs/msg/mesh.hpp>
#include <vector>
#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include <queue>
#include "delaunay.h"

using Point = CGAL::Epick::Point_3;

// Converts geometry_msgs::msg::Point to CGAL::Epick::Point_3
std::vector<Point> convertPointsToCGAL(const std::vector<geometry_msgs::msg::Point> &points) {
	return points | ranges::views::transform([](const geometry_msgs::msg::Point &p) {
		return Point(p.x, p.y, p.z);
	}) | ranges::to_vector;
}

std::vector<CGAL::Epick::Point_3> surface_points_on_mesh(const shape_msgs::msg::Mesh &mesh, double min_radius = 0.1) {

	auto points = convertPointsToCGAL(mesh.vertices);

	for (const auto &triangle: mesh.triangles) {
		const Point &p1 = points[triangle.vertex_indices[0]];
		const Point &p2 = points[triangle.vertex_indices[1]];
		const Point &p3 = points[triangle.vertex_indices[2]];

		if (CGAL::collinear(p1, p2, p3)) {
			continue;
		}

		double outcircle_radius = std::sqrt(CGAL::squared_radius(p1, p2, p3));

		if (outcircle_radius > min_radius) {
			points.push_back(CGAL::barycenter(p1, 1.0, p2, 1.0, p3, 1.0));
		}
	}

	return points;
}

CGAL::Delaunay_triangulation_3<CGAL::Epick>
utilities::generateDelaunayTriangulation(const shape_msgs::msg::Mesh &mesh) {
	auto points = surface_points_on_mesh(mesh);
	CGAL::Delaunay_triangulation_3<CGAL::Epick> dt(points.begin(), points.end());
	return dt;
}

utilities::Triangle utilities::facet_triangle(const Delaunay3::Cell_handle &cell, int facet_index) {
	return Triangle(
			cell->vertex((facet_index + 1) % 4)->point(),
			cell->vertex((facet_index + 2) % 4)->point(),
			cell->vertex((facet_index + 3) % 4)->point()
	);
}
