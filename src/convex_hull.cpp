

#include "convex_hull.h"

// Warning: Qhull headers seem to conflict with some of the other headers, notably rangev3.
// So, keep these below the other imports, and avoid including more than the absolute minimum in this file.
#include <libqhullcpp/RboxPoints.h>
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacetList.h>
#include <libqhullcpp/QhullVertexSet.h>

shape_msgs::msg::Mesh convexHull(const std::vector<geometry_msgs::msg::Point> &mesh_points) {

	orgQhull::RboxPoints points;

	for (const auto& p : mesh_points) {
		double coords[3] = {p.x, p.y, p.z};

		orgQhull::QhullPoint qhp(3, coords);

		points.append(qhp);
	}

	orgQhull::Qhull qhull;
	qhull.runQhull(points, "");

	shape_msgs::msg::Mesh mesh;

	for (const auto& facet : qhull.facetList()) {

		assert(facet.vertices().size() == 3);

		shape_msgs::msg::MeshTriangle t;
		t.vertex_indices[0] = facet.vertices().at(0).id();
		t.vertex_indices[1] = facet.vertices().at(1).id();
		t.vertex_indices[2] = facet.vertices().at(2).id();
		mesh.triangles.push_back(t);
	}

	for (const auto& vertex : qhull.vertexList()) {
		geometry_msgs::msg::Point p;
		p.x = vertex.point()[0];
		p.y = vertex.point()[1];
		p.z = vertex.point()[2];
		mesh.vertices.push_back(p);
	}
	return mesh;
}