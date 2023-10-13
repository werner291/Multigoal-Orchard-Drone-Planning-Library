// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/transform.hpp>
#include "../TreeMeshes.h"
#include "../utilities/convex_hull.h"
#include "../utilities/mesh_utils.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/Random.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <range/v3/view/take.hpp>

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Triangle_mesh = CGAL::Surface_mesh<Kernel::Point_3>;
using Traits = CGAL::Surface_mesh_shortest_path_traits<Kernel, Triangle_mesh>;
using Primitive = CGAL::AABB_face_graph_triangle_primitive<Triangle_mesh>;
using AABBTraits = CGAL::AABB_traits<Kernel, Primitive>;
using Surface_mesh_shortest_path = CGAL::Surface_mesh_shortest_path<Traits>;
using CGALMeshPoint = Surface_mesh_shortest_path::Face_location;

Triangle_mesh from_rosmsg(const shape_msgs::msg::Mesh &mesh) {

	Triangle_mesh tmesh;

	// We first convert every vertex to a CGAL point, building a vector to translate between the original and CGAL point indices.
	std::vector<Triangle_mesh::vertex_index> vertices;

	for (auto &v: mesh.vertices) {
		vertices.push_back(tmesh.add_vertex(Kernel::Point_3(v.x, v.y, v.z)));
	}

	// Now we add every triangle to the mesh.
	for (auto &f: mesh.triangles) {
		Triangle_mesh::face_index fi = tmesh.add_face(vertices[f.vertex_indices[0]],
													  vertices[f.vertex_indices[1]],
													  vertices[f.vertex_indices[2]]);
		assert(fi != Triangle_mesh::null_face());
	}

	return tmesh;
}

std::vector<double> distanceMatrix1(Triangle_mesh tmesh, const std::vector<Eigen::Vector3d> &apples) {

	/// An AABB-tree for quick lookup of the on_which_mesh point on the mesh (including facet information)
	CGAL::AABB_tree<AABBTraits> aabbtree{};

	// We initialize the AABB tree such that we don't have to re-compute it every projection query.
	Surface_mesh_shortest_path shortest_paths(tmesh);
	shortest_paths.build_aabb_tree(aabbtree);

	std::vector<double> distances;
	distances.reserve(apples.size() * apples.size());

	// Loop over every pair of apples in an NxN grid
	for (const auto &apple1: apples) {
		for (const auto &apple2: apples) {

			// Find the closest point on the mesh to the apple
			CGALMeshPoint p1 = shortest_paths.locate(Kernel::Point_3(apple1.x(), apple1.y(), apple1.z()), aabbtree);
			CGALMeshPoint p2 = shortest_paths.locate(Kernel::Point_3(apple2.x(), apple2.y(), apple2.z()), aabbtree);

			// Add pur start point a
			shortest_paths.add_source_point(p1);

			// Compute the path to a from b.
			auto result = shortest_paths.shortest_distance_to_source_points(p2.first, p2.second);

			distances.push_back(result.first);
		}
	}

	return distances;
}

std::vector<double> distanceMatrix2(Triangle_mesh tmesh, const std::vector<Eigen::Vector3d> &apples) {

	/// An AABB-tree for quick lookup of the on_which_mesh point on the mesh (including facet information)
	CGAL::AABB_tree<AABBTraits> aabbtree{};

	// We initialize the AABB tree such that we don't have to re-compute it every projection query.
	Surface_mesh_shortest_path shortest_paths(tmesh);
	shortest_paths.build_aabb_tree(aabbtree);

	std::vector<double> distances;
	distances.reserve(apples.size() * apples.size());

	// Loop over every pair of apples in an NxN grid
	for (const auto &apple1: apples) {

		// Find the closest point on the mesh to the apple
		CGALMeshPoint p1 = shortest_paths.locate(Kernel::Point_3(apple1.x(), apple1.y(), apple1.z()), aabbtree);

		// Add our start point a
		shortest_paths.add_source_point(p1);

		for (const auto &apple2: apples) {

			CGALMeshPoint p2 = shortest_paths.locate(Kernel::Point_3(apple2.x(), apple2.y(), apple2.z()), aabbtree);

			// Compute the path to a from b.
			auto result = shortest_paths.shortest_distance_to_source_points(p2.first, p2.second);

			distances.push_back(result.first);
		}
	}

	return distances;
}


int main() {

	TreeMeshes tree = loadTreeMeshes("appletree3");

	auto hull = convexHull(tree.leaves_mesh.vertices);

	const std::vector<Eigen::Vector3d> apples =
			tree.fruit_meshes | ranges::views::transform([](const auto &mesh) -> Eigen::Vector3d {
				return mesh_aabb(mesh).center();
			}) | ranges::to_vector;

	Triangle_mesh tmesh = from_rosmsg(hull);

	// List of apple amounts to process
	std::vector<int> appleAmounts = {10, 20, 50, 100, 200, 500};

	for (int amount: appleAmounts) {

		std::cout << "Number of apples: " << amount << std::endl;

		auto apples_subset = apples | ranges::views::take(amount) | ranges::to_vector;

		auto start = std::chrono::high_resolution_clock::now();
		auto m1 = distanceMatrix1(tmesh, apples_subset);
		auto end = std::chrono::high_resolution_clock::now();
		double distance_sum = std::accumulate(m1.begin(), m1.end(), 0.0);

		std::cout << "Time 1: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
				  << std::endl;
		std::cout << "Distance sum 1: " << distance_sum << std::endl;


		auto start2 = std::chrono::high_resolution_clock::now();
		auto m2 = distanceMatrix2(tmesh, apples_subset);
		auto end2 = std::chrono::high_resolution_clock::now();
		double distance_sum2 = std::accumulate(m2.begin(), m2.end(), 0.0);

		std::cout << "Time 2: " << std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start2).count()
				  << std::endl;
		std::cout << "Distance sum 2: " << distance_sum2 << std::endl;

	}

}