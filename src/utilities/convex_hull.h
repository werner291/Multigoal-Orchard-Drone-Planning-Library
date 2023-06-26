
#ifndef NEW_PLANNERS_CONVEX_HULL_H
#define NEW_PLANNERS_CONVEX_HULL_H

// Warning: Avoid including Qhull in the header, as it apparently conflicts
// with some of the other headers we're using throughout the codebase.

#include <geometry_msgs/msg/point.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

/**
 * Construct a triangulated convex hull mesh from the given vector of points.
 *
 * Note that this method can easily be used on triangle meshes as well by simply
 * giving it the `vertices` field of the mesh message.
 *
 * @param mesh_points 	The points to construct the convex hull from.
 * @return 				The convex hull mesh.
 */
shape_msgs::msg::Mesh convexHull(const std::vector<geometry_msgs::msg::Point> &mesh_points);

namespace mgodpl::chull_tools {

	using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
	using Triangle_mesh = CGAL::Surface_mesh<Kernel::Point_3>;

	// Function to compute and return the convex hull of the given points as a Triangle_mesh
	Triangle_mesh computeConvexHullAsMesh(const std::vector<Kernel::Point_3>& points);

}

#endif //NEW_PLANNERS_CONVEX_HULL_H
