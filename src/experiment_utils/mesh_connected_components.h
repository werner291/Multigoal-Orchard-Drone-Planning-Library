// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/13/23.
//

#ifndef MGODPL_MESH_CONNECTED_COMPONENTS_H
#define MGODPL_MESH_CONNECTED_COMPONENTS_H

#include <vector>
#include <cstddef>
#include <shape_msgs/msg/mesh.hpp>

/**
 * Discovers the connected components of the mesh.
 * Output is a vector of vectors of vertex indices.
 * Each vector is a connected component.
 */
std::vector<std::vector<size_t>> connected_vertex_components(const shape_msgs::msg::Mesh &mesh);

/**
 * Given a mesh, return a vector of meshes where each mesh is a connected component of the original mesh,
 * where two vertices are connected if they are connected by at least one triangle.
 *
 * @param combined_mesh 		The mesh to split into connected components.
 * @return 						A vector of meshes, where each mesh is a connected component of the original mesh.
 */
std::vector<shape_msgs::msg::Mesh> break_down_to_connected_components(const shape_msgs::msg::Mesh &combined_mesh);

#endif //MGODPL_MESH_CONNECTED_COMPONENTS_H
