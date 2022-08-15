

#include "convex_hull.h"
#include "general_utilities.h"

// Warning: Qhull headers seem to conflict with some other headers, notably rangev3.
// So, keep these below the other imports, and avoid including more than the absolute minimum in this file.
#include <libqhullcpp/RboxPoints.h>
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacetList.h>
#include <libqhullcpp/QhullVertexSet.h>
#include <Eigen/Core>

shape_msgs::msg::Mesh convexHull(const std::vector<geometry_msgs::msg::Point> &mesh_points) {

	orgQhull::RboxPoints points;

	for (const auto& p : mesh_points) {
		double coords[3] = {p.x, p.y, p.z};

		orgQhull::QhullPoint qhp(3, coords);

		points.append(qhp);
	}

	orgQhull::Qhull qhull;
	qhull.runQhull(points, "Qt");

	shape_msgs::msg::Mesh mesh;

	std::vector<size_t> point_id_translation(mesh_points.size(), SIZE_MAX);

	for (const auto& vertex : qhull.vertexList()) {
		geometry_msgs::msg::Point p;
		p.x = vertex.point()[0];
		p.y = vertex.point()[1];
		p.z = vertex.point()[2];
		mesh.vertices.push_back(p);

		point_id_translation[vertex.point().id()] = mesh.vertices.size() - 1;
	}

	for (const auto& facet : qhull.facetList()) {

		assert(facet.vertices().size() == 3);

		shape_msgs::msg::MeshTriangle t;
		t.vertex_indices[0] = point_id_translation[facet.vertices().at(0).point().id()];
		t.vertex_indices[1] = point_id_translation[facet.vertices().at(1).point().id()];
		t.vertex_indices[2] = point_id_translation[facet.vertices().at(2).point().id()];
		mesh.triangles.push_back(t);

		assert(t.vertex_indices[0] < mesh.vertices.size());
		assert(t.vertex_indices[1] < mesh.vertices.size());
		assert(t.vertex_indices[2] < mesh.vertices.size());

	}

	fixWinding(mesh);

	return mesh;
}
