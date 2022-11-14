//
// Created by werner on 11-11-22.
//

#include "BruteForceDedupStreamingAlphashape.h"

void BruteForceDedupStreamingAlphashape::addPoints(const std::vector<Eigen::Vector3d> &points) {

	for (const auto& point : points) {
		if (dedup.any_within(point, threshold)) {
			continue;
		} else {
			dedup.insert(point, std::monostate());

			cgal_points.emplace_back(point.x(), point.y(), point.z());
		}
	}

}

BruteForceDedupStreamingAlphashape::BruteForceDedupStreamingAlphashape(double threshold) : threshold(threshold), dedup(threshold, 1000) {

}

shape_msgs::msg::Mesh BruteForceDedupStreamingAlphashape::toMesh() {

	Delaunay dt;

	for (const auto& point : cgal_points) {
		dt.insert(point);
	}

	Alpha_shape_3 as(dt);

	std::vector<Delaunay::Facet> facets;
	as.get_alpha_shape_facets(std::back_inserter(facets), Alpha_shape_3::REGULAR, 1.0);

	shape_msgs::msg::Mesh mesh;

	std::unordered_map<Delaunay::Vertex_handle, int> vertex_to_index;

	for (const auto& facet : facets) {

		shape_msgs::msg::MeshTriangle triangle;

		for (int i = 0; i < 3; i++) {

			if (i == facet.second) {
				// I don't know what "second" is? Is it ever not 3? Was suggested by copilot, but I don't understand it.
				// TODO figure it out, or if it's just gibberish, remove it.
				continue;
			}

			auto vertex = facet.first->vertex(i);

			if (vertex_to_index.find(vertex) == vertex_to_index.end()) {
				geometry_msgs::msg::Point point;
				point.x = vertex->point().x();
				point.y = vertex->point().y();
				point.z = vertex->point().z();

				mesh.vertices.push_back(point);

				vertex_to_index[vertex] = mesh.vertices.size() - 1;
			}

			triangle.vertex_indices[i] = vertex_to_index[vertex];
		}

		mesh.triangles.push_back(triangle);
	}

	throw std::runtime_error("Not implemented");

}
