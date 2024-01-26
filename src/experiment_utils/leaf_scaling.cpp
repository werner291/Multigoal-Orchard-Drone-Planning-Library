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

namespace mgodpl {

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
	std::vector<Triangle> createTrianglesFromMesh(const shape_msgs::msg::Mesh &mesh) {
		std::vector<Triangle> triangles;
		for (const auto &face: mesh.triangles) {
			const auto &vertex1 = mesh.vertices[face.vertex_indices[0]];
			const auto &vertex2 = mesh.vertices[face.vertex_indices[1]];
			const auto &vertex3 = mesh.vertices[face.vertex_indices[2]];
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
	AABB_Tree buildAABBTree(const shape_msgs::msg::Mesh &mesh) {
		std::vector<Triangle> triangles = createTrianglesFromMesh(mesh);
		AABB_Tree tree(triangles.begin(), triangles.end());
		tree.build();
		return tree;
	}

	/**
	 * @brief Finds the leaf vertex that is closest to the trunk and the corresponding closest point on the trunk.
	 *
	 * This function iterates over all the leaf vertices in the given component. For each leaf vertex, it queries the AABB tree
	 * to find the closest point on the trunk. It keeps track of the leaf vertex that is closest to the trunk and the corresponding
	 * closest point on the trunk.
	 *
	 * @param tree The AABB tree representing the trunk.
	 * @param component The connected component of leaf vertices to consider.
	 * @param leaves_mesh The mesh representing the leaves.
	 * @return A pair consisting of the index of the leaf vertex that is closest to the trunk and a math::Vec3d representing the
	 * closest point on the trunk.
	 */
	std::pair<size_t, math::Vec3d> findLeafVertexClosestToTrunk(const AABB_Tree &tree,
																const std::vector<size_t> &component,
																const shape_msgs::msg::Mesh &leaves_mesh) {
		// Initialize the index of the closest leaf vertex and the minimum distance to infinity.
		size_t closest_leaf_index = 0;
		double closest_leaf_distance = std::numeric_limits<double>::infinity();
		Point closest_point;

		// Iterate over all the leaf vertices in the component.
		for (const auto &leaf_index: component) {
			// Get the current leaf vertex.
			const auto &leaf_vertex = leaves_mesh.vertices[leaf_index];
			Point query_point(leaf_vertex.x, leaf_vertex.y, leaf_vertex.z);

			// Query the AABB tree to find the closest point on the trunk to the current leaf vertex.
			auto result = tree.closest_point_and_primitive(query_point);
			double distance = CGAL::squared_distance(query_point, result.first);

			// If the current leaf vertex is closer to the trunk than the previous closest leaf vertex, update the closest leaf vertex and the minimum distance.
			if (distance < closest_leaf_distance) {
				closest_leaf_distance = distance;
				closest_leaf_index = leaf_index;
				closest_point = result.first;
			}
		}

		// Return the index of the closest leaf vertex and the closest point on the trunk.
		return {closest_leaf_index, math::Vec3d(closest_point.x(), closest_point.y(), closest_point.z())};
	}

	/**
	 * @brief Calculates the root point for each leaf in the tree.
	 *
	 * This function finds, for each leaf, the closest point on the trunk and assigns it
	 * as the root point for all the leaf vertices in the same connected component.
	 *
	 * @param tree_meshes The tree meshes which include the trunk mesh and the leaves mesh.
	 * @return A vector of math::Vec3d objects representing the root point for each leaf.
	 */
	std::vector<math::Vec3d> leaf_root_points(const mgodpl::tree_meshes::TreeMeshes &tree_meshes) {
		// Build an AABB tree from the trunk mesh
		AABB_Tree tree = buildAABBTree(tree_meshes.trunk_mesh);

		// Initialize a vector to store the root point for each leaf
		std::vector<math::Vec3d> leaf_root_vertex(tree_meshes.leaves_mesh.vertices.size(), math::Vec3d(0, 0, 0));

		// Iterate over all the connected components of the leaves mesh
		for (const auto &component: connected_vertex_components(tree_meshes.leaves_mesh)) {
			// Find the leaf vertex that is closest to the trunk and the corresponding closest point on the trunk
			auto [closest_leaf_index, closest_point] = findLeafVertexClosestToTrunk(tree, component, tree_meshes.leaves_mesh);

			// Assign the closest point as the root point for all the leaf vertices in the current component
			for (const auto &leaf_index: component) {
				leaf_root_vertex[leaf_index] = closest_point;
			}
		}

		// Return the vector of root points
		return leaf_root_vertex;
	}

	shape_msgs::msg::Mesh scale_leaves(const mgodpl::tree_meshes::TreeMeshes &tree_meshes,
											   const std::vector<math::Vec3d> &leaf_root_vertex,
											   double scale_factor) {
		// Create a copy of the leaves mesh
		shape_msgs::msg::Mesh leaves_mesh_copy = tree_meshes.leaves_mesh;

		// Iterate over all the vertices in the mesh
		for (size_t i = 0; i < leaves_mesh_copy.vertices.size(); ++i) {
			// Retrieve the root vertex for the current leaf vertex
			const auto &scale_center = leaf_root_vertex[i];

			// Create a math::Vec3d object representing the current leaf vertex
			math::Vec3d leaf_vertex(leaves_mesh_copy.vertices[i].x,
									leaves_mesh_copy.vertices[i].y,
									leaves_mesh_copy.vertices[i].z);

			// Scale the leaf vertex around the root point by the given scale factor
			leaf_vertex = scale_center + (leaf_vertex - scale_center) * scale_factor;

			// Write the scaled vertex back to the mesh
			leaves_mesh_copy.vertices[i].x = leaf_vertex.x();
			leaves_mesh_copy.vertices[i].y = leaf_vertex.y();
			leaves_mesh_copy.vertices[i].z = leaf_vertex.z();
		}

		// Return the modified leaves mesh
		return leaves_mesh_copy;
	}
}