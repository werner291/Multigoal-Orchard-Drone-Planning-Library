
#include "StreamingAlphaShape.h"

StreamingAlphaShape::StreamingAlphaShape(double margin) {



}

void StreamingAlphaShape::addPoint(const Eigen::Vector3d &point) {

	// Convert to CGAL point.
	Point p(point.x(), point.y(), point.z());

	// Insert the point into the Delaunay triangulation.

	// The algorithm (implemented in CGAL) will locate the (possibly infinite) cell the point falls into,
	// delete it, and replace it with four new tetrahedral cells that have this point as a vertex, and the
	// faces of the original as the opposite face.
	//
	// To maintain the Delaunay condition, faces will be flipped as needed
	//
	auto vertex = dt.insert(p);

	std::vector<Delaunay::Cell_handle> cells;
	dt.incident_cells(vertex, std::back_inserter(cells));

	for (auto cell : cells) {

		// If this cell has a circumsphere with radius less than alpha,
		// then one facet must be on the boundary of the alpha shape. (Not correct?)

		// TODO also check if any of the facets are on the boundary of the alpha shape.

		dt.incident_facets(cell)

	}

}

shape_msgs::msg::Mesh StreamingAlphaShape::extractSurfaceMesh() {

	std::unordered_map<Delaunay::Vertex_handle, size_t> vertexIndices;

	shape_msgs::msg::Mesh mesh;

	for (const auto &tri: this->triangles) {

		for (auto vertex : {tri.v1, tri.v2, tri.v3}) {

			if (vertexIndices.find(vertex) == vertexIndices.end()) {

				geometry_msgs::msg::Point point;
				point.x = dt.point(vertex).x();
				point.y = dt.point(vertex).y();
				point.z = dt.point(vertex).z();

				mesh.vertices.push_back(point);

				vertexIndices[vertex] = mesh.vertices.size() - 1;

			}

		}

		shape_msgs::msg::MeshTriangle triangle;
		triangle.vertex_indices[0] = vertexIndices[tri.v1];
		triangle.vertex_indices[1] = vertexIndices[tri.v2];
		triangle.vertex_indices[2] = vertexIndices[tri.v3];

		mesh.triangles.push_back(triangle);

	}


}