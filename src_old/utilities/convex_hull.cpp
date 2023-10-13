

#include "convex_hull.h"
#include "general_utilities.h"
#include "msgs_utilities.h"

// Warning: Qhull headers seem to conflict with some other headers, notably rangev3.
// So, keep these below the other imports, and avoid including more than the absolute minimum in this file.
#include <libqhullcpp/RboxPoints.h>
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacetList.h>
#include <libqhullcpp/QhullVertexSet.h>

shape_msgs::msg::Mesh convexHull(const std::vector<geometry_msgs::msg::Point> &mesh_points) {

	int pointDimension = 3;
	int pointCount = mesh_points.size();

	// Allocate an array to hold all the point coordinates
	std::vector<realT> pointCoordinates(pointCount * pointDimension);

	// Fill the array with the point coordinates
	for (size_t i = 0; i < mesh_points.size(); ++i) {
		pointCoordinates[i * pointDimension] = mesh_points[i].x;
		pointCoordinates[i * pointDimension + 1] = mesh_points[i].y;
		pointCoordinates[i * pointDimension + 2] = mesh_points[i].z;
	}

	// Create a Qhull object
	orgQhull::Qhull qhull;

	// Run Qhull
	qhull.runQhull("", pointDimension, pointCount, pointCoordinates.data(), "Qt");

	shape_msgs::msg::Mesh mesh;

	std::vector<size_t> point_id_translation(mesh_points.size(), SIZE_MAX);

	for (const auto &vertex: qhull.vertexList()) {
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

CGAL::Surface_mesh<mgodpl::chull_tools::Kernel::Point_3>
mgodpl::chull_tools::computeConvexHullAsMesh(const std::vector<Kernel::Point_3> &points) {
	mgodpl::chull_tools::Triangle_mesh convexHull;
	CGAL::convex_hull_3(points.begin(), points.end(), convexHull);
	return convexHull;
}
