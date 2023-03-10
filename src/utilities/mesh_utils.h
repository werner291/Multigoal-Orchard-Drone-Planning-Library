
// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_MESH_UTILS_H
#define NEW_PLANNERS_MESH_UTILS_H

#include <Eigen/Core>
#include <shape_msgs/msg/mesh.hpp>
#include "general_utilities.h"
#include "msgs_utilities.h"

/**
 * Find the on_which_mesh point on the surface of the given mesh to the given point.
 *
 * Takes O(n) in the number of triangles in the mesh.
 *
 * @param mesh 			The mesh to find the on_which_mesh point on.
 * @param query_point 	The point to find the on_which_mesh point to.
 * @return 				The on_which_mesh point on the mesh to the given point.
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

/**
 * Given a vector of meshes, return a single mesh combining all the meshes in the vector.
 *
 * Guarantee: the meshes will be added in order, so the IDs of the vertices will be the same as the indices of the vertices
 * in the vector, plus the number of vertices in all the meshes before it.
 *
 * @param meshes 		The meshes to combine.
 * @return 				A single mesh combining all the meshes in the vector.
 */
shape_msgs::msg::Mesh combine_meshes(const std::vector<shape_msgs::msg::Mesh> &meshes);

/**
 * Compute the AABB of the vertices of the given mesh.
 *
 * @param mesh 		The mesh to compute the AABB of.
 *
 * @return 			The AABB of the vertices of the given mesh.
 */
Eigen::AlignedBox3d mesh_aabb(const shape_msgs::msg::Mesh &mesh);

/**
 * @brief Creates a flat ground plane mesh centered at the origin with the specified width and height.
 *
 * The ground plane mesh is created with four vertices and two triangles forming a rectangle.
 *
 * @param width The width of the ground plane.
 * @param height The height of the ground plane.
 * @return A shape_msgs::msg::Mesh representing the ground plane.
 */
shape_msgs::msg::Mesh createGroundPlane(double width, double height);


#endif //NEW_PLANNERS_MESH_UTILS_H
