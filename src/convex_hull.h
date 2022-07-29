
#ifndef NEW_PLANNERS_CONVEX_HULL_H
#define NEW_PLANNERS_CONVEX_HULL_H

// Warning: Avoid including Qhull in the header, as it apparently conflicts
// with some of the other headers we're using throughout the codebase.
#include <geometry_msgs/msg/point.hpp>
#include <shape_msgs/msg/mesh.hpp>

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

#endif //NEW_PLANNERS_CONVEX_HULL_H
