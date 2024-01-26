// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 26-1-24.
//

#include "leaf_scaling.h"
#include "mesh_connected_components.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

using K = CGAL::Simple_cartesian<double>;
using Point = K::Point_3;
using Triangle = K::Triangle_3;
using Iterator = std::vector<Triangle>::iterator;
using Primitive = CGAL::AABB_triangle_primitive<K, Iterator>;
using AABB_triangle_traits = CGAL::AABB_traits<K, Primitive>;
using AABB_Tree = CGAL::AABB_tree<AABB_triangle_traits>;

/**
 * @brief Converts a ROS shape_msgs::Mesh into a vector of CGAL Triangles.
 *
 * This function iterates over the triangles in the given mesh, and for each triangle,
 * it retrieves the corresponding vertices and creates a CGAL Triangle. All the triangles
 * are stored in a vector which is returned by the function.
 *
 * @param mesh The ROS shape_msgs::Mesh to convert.
 * @return A vector of CGAL Triangles representing the given mesh.
 */
std::vector<Triangle> createTrianglesFromMesh(const shape_msgs::msg::Mesh& mesh) {
    std::vector<Triangle> triangles;
    for (const auto& face : mesh.triangles) {
        const auto& vertex1 = mesh.vertices[face.vertex_indices[0]];
        const auto& vertex2 = mesh.vertices[face.vertex_indices[1]];
        const auto& vertex3 = mesh.vertices[face.vertex_indices[2]];
        triangles.emplace_back(Point(vertex1.x, vertex1.y, vertex1.z),
                               Point(vertex2.x, vertex2.y, vertex2.z),
                               Point(vertex3.x, vertex3.y, vertex3.z));
    }
    return triangles;
}

/**
 * @brief Builds an AABB tree from a given mesh.
 *
 * This function first converts the given mesh into a vector of CGAL Triangles using the createTrianglesFromMesh function.
 * Then, it creates an AABB tree from the vector of triangles and builds the tree.
 *
 * @param mesh The ROS shape_msgs::Mesh to convert into an AABB tree.
 * @return An AABB tree representing the given mesh.
 */
AABB_Tree buildAABBTree(const shape_msgs::msg::Mesh& mesh) {
    std::vector<Triangle> triangles = createTrianglesFromMesh(mesh);
    AABB_Tree tree(triangles.begin(), triangles.end());
    tree.build();
    return tree;
}

/**
 * @brief Finds the leaf closest to the trunk for each connected component.
 *
 * This function iterates over each leaf in the given component and calculates the distance from the leaf to the trunk.
 * The leaf with the smallest distance is considered the closest leaf.
 *
 * @param tree The AABB tree representing the trunk.
 * @param component The connected component to find the closest leaf for.
 * @param leaves_mesh The mesh representing the leaves.
 * @return The index of the leaf closest to the trunk.
 */
size_t findClosestLeaf(const AABB_Tree& tree, const std::vector<size_t>& component, const shape_msgs::msg::Mesh& leaves_mesh) {
    size_t closest_leaf_index = 0;
    double closest_leaf_distance = std::numeric_limits<double>::infinity();

    for (const auto& leaf_index : component){
        const auto& leaf_vertex = leaves_mesh.vertices[leaf_index];
        Point query_point(leaf_vertex.x, leaf_vertex.y, leaf_vertex.z);
        auto result = tree.closest_point_and_primitive(query_point);
        double distance = CGAL::squared_distance(query_point, result.first);
        if (distance < closest_leaf_distance){
            closest_leaf_distance = distance;
            closest_leaf_index = leaf_index;
        }
    }

    return closest_leaf_index;
}

/**
 * @brief Assigns the closest leaf to all vertices in the connected component.
 *
 * This function iterates over each leaf in the given component and assigns the index of the closest leaf to the leaf.
 *
 * @param leaf_root_vertex The vector to store the index of the closest leaf for each leaf in the component.
 * @param component The connected component to assign the closest leaf for.
 * @param closest_leaf_index The index of the leaf closest to the trunk.
 */
void assignClosestLeafToComponent(std::vector<size_t>& leaf_root_vertex, const std::vector<size_t>& component, size_t closest_leaf_index) {
    for (const auto& leaf_index : component){
        leaf_root_vertex[leaf_index] = closest_leaf_index;
    }
}


std::vector<size_t> mgodpl::leaf_root_vertex(const mgodpl::tree_meshes::TreeMeshes &tree_meshes) {
	AABB_Tree tree = buildAABBTree(tree_meshes.trunk_mesh);

	std::vector<size_t> leaf_root_vertex(tree_meshes.leaves_mesh.vertices.size());

	for (const auto& component : connected_vertex_components(tree_meshes.leaves_mesh)) {
		size_t closest_leaf_index = findClosestLeaf(tree, component, tree_meshes.leaves_mesh);
		assignClosestLeafToComponent(leaf_root_vertex, component, closest_leaf_index);
	}

	return leaf_root_vertex;
}
