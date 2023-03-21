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

utilities::Delaunay3::Cell_handle
utilities::closest_cell(const Delaunay3::Point to, std::vector<Delaunay3::Cell_handle> &big_cells) {
	double distance = std::numeric_limits<double>::max();
	CGAL::Delaunay_triangulation_3<::CGAL::Epick, ::CGAL::Default, ::CGAL::Default, ::CGAL::Default>::Cell_handle closest_cell;

	for (const auto &cell: big_cells) {

		Delaunay3::Point pt1 = cell->vertex(0)->point();
		Delaunay3::Point pt2 = cell->vertex(1)->point();
		Delaunay3::Point pt3 = cell->vertex(2)->point();
		Delaunay3::Point pt4 = cell->vertex(3)->point();

		Delaunay3::Point barycenter = CGAL::centroid(pt1, pt2, pt3, pt4);

		double d = CGAL::squared_distance(to, barycenter);

		if (d < distance) {
			distance = d;
			closest_cell = cell;
		}

	}

	return closest_cell;
}

std::vector<utilities::Delaunay3::Cell_handle> utilities::bidirectional_monotonic_astar(Delaunay3::Cell_handle start,
																						Delaunay3::Cell_handle goal,
																						std::vector<Delaunay3::Cell_handle> &big_cells) {

	struct FrontierCell {
		std::shared_ptr<FrontierCell> parent;
		double heuristic_distance;
		Delaunay3::Cell_handle cell;
	};

	std::priority_queue<std::shared_ptr<FrontierCell>> frontier{

			std::make_shared<FrontierCell>(nullptr, 0.0)


	};


}

/*
 *
	shape_msgs::msg::Mesh mesh;
	shape_msgs::msg::Mesh mesh_outer;

	for (auto itr = dt.cells_begin(); itr != dt.cells_end(); ++itr) {

		Point circumcenter = itr->circumcenter();

		double sphere_radius_sqr = squared_distance(circumcenter, itr->vertex(0)->point());

		if (sphere_radius_sqr < min_radius * min_radius) {
			continue;
		}

		bool cutoff = false;

		for (int i = 0; i < 4; ++i) {
			const auto &p = itr->vertex(i)->point();
			if (p.x() < 0.0 || p.y() < 0.0) {
				cutoff = true;
				break;
			}
		}

		if (cutoff) {
			continue;
		}

		if (!dt.is_infinite(itr)) {

			size_t vertex_index_base = mesh.vertices.size();

			for (int i = 0; i < 4; ++i) {
				const auto &p = itr->vertex(i)->point();
				geometry_msgs::msg::Point pt;
				pt.x = p.x();
				pt.y = p.y();
				pt.z = p.z();

				assert(isfinite(pt.x) && isfinite(pt.y) && isfinite(pt.z));

				mesh.vertices.emplace_back(std::move(pt));
			}

			shape_msgs::msg::MeshTriangle tri;
			tri.vertex_indices = {vertex_index_base + 0, vertex_index_base + 1, vertex_index_base + 2};
			mesh.triangles.emplace_back(tri);
			tri.vertex_indices = {vertex_index_base + 0, vertex_index_base + 2, vertex_index_base + 3};
			mesh.triangles.emplace_back(tri);
			tri.vertex_indices = {vertex_index_base + 0, vertex_index_base + 3, vertex_index_base + 1};
			mesh.triangles.emplace_back(tri);
			tri.vertex_indices = {vertex_index_base + 1, vertex_index_base + 3, vertex_index_base + 2};
			mesh.triangles.emplace_back(tri);

		} else {

			size_t vertex_index_base = mesh_outer.vertices.size();

			for (int i = 0; i < 4; ++i) {
				const auto &p = itr->vertex(i)->point();
				if (dt.is_infinite(itr->vertex(i))) {
					geometry_msgs::msg::Point pt;
					pt.x = p.x();
					pt.y = p.y();
					pt.z = p.z();
					mesh_outer.vertices.emplace_back(std::move(pt));
				}
			}

			shape_msgs::msg::MeshTriangle tri;
			tri.vertex_indices = {vertex_index_base + 0, vertex_index_base + 1, vertex_index_base + 2};
			mesh_outer.triangles.emplace_back(tri);

		}

	}

	viewer.addMesh(mesh, {1.0, 0.0, 1.0}, 1.0);
	viewer.addMesh(mesh_outer, {0.0, 1.0, 1.0}, 1.0);
 */
