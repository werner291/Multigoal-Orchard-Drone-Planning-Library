
// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_MESH_UTILS_H
#define NEW_PLANNERS_MESH_UTILS_H

//#include <Eigen/Core>
#include <shape_msgs/msg/mesh.hpp>
#include "../math/AABB.h"

///**
// * Find the on_which_mesh point on the surface of the given mesh to the given point.
// *
// * Takes O(n) in the number of triangles in the mesh.
// *
// * @param mesh 			The mesh to find the on_which_mesh point on.
// * @param query_point 	The point to find the on_which_mesh point to.
// * @return 				The on_which_mesh point on the mesh to the given point.
// */
//Eigen::Vector3d closestPointOnMesh(const shape_msgs::msg::Mesh &mesh, const Eigen::Vector3d &query_point);



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
mgodpl::math::AABBd mesh_aabb(const shape_msgs::msg::Mesh &mesh);

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

/**
 * @brief Merges the right-hand mesh into the left-hand mesh in-place.
 *
 * This function takes two input meshes (left_mesh and right_mesh) and combines the vertices and triangles
 * of the right-hand mesh into the left-hand mesh. The vertex indices in the triangles are adjusted to account
 * for the new vertex ordering in the merged mesh.
 *
 * @param left_mesh Reference to the first input mesh (left-hand side) which will be modified in-place.
 * @param right_mesh The second input mesh (right-hand side) to be merged into the left-hand mesh.
 */
void append_mesh(shape_msgs::msg::Mesh &left_mesh, const shape_msgs::msg::Mesh &right_mesh);

/**
 * @brief Extracts vertices from a given mesh and returns them as a vector of 3-length arrays of Vec3d.
 *
 * @param leaves_mesh The mesh from which to extract the vertices. It is of type shape_msgs::msg::Mesh.
 * @return A vector of 3-length arrays, each representing a triangle in the mesh.
 */
std::vector<std::array<mgodpl::math::Vec3d,3>> triangles_from_mesh(const shape_msgs::msg::Mesh& leaves_mesh);

#endif //NEW_PLANNERS_MESH_UTILS_H
