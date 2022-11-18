
// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_MESH_UTILS_H
#define NEW_PLANNERS_MESH_UTILS_H

#include <Eigen/Core>
#include <shape_msgs/msg/mesh.hpp>
#include "general_utilities.h"

/**
 * Find the closest point on the surface of the given mesh to the given point.
 *
 * Takes O(n) in the number of triangles in the mesh.
 *
 * @param mesh 			The mesh to find the closest point on.
 * @param query_point 	The point to find the closest point to.
 * @return 				The closest point on the mesh to the given point.
 */
Eigen::Vector3d closestPointOnMesh(const shape_msgs::msg::Mesh &mesh, const Eigen::Vector3d &query_point);

/**
 * Given a mesh, return a vector of meshes where each mesh is a connected component of the original mesh,
 * where two vertices are connected if they are connected by at least one triangle.
 *
 * @param combined_mesh 		The mesh to split into connected components.
 * @return 						A vector of meshes, where each mesh is a connected component of the original mesh.
 */
std::vector<shape_msgs::msg::Mesh> break_down_to_connected_components(const shape_msgs::msg::Mesh &combined_mesh);

#endif //NEW_PLANNERS_MESH_UTILS_H
