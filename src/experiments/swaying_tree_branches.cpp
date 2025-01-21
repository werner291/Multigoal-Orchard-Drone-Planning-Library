// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#include <ranges>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <CGAL/extract_mean_curvature_flow_skeleton.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/fair.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>

#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/default_colors.h"
#include "../math/Quaternion.h"
#include "../visualization/VtkLineSegmentVizualization.h"
#include "../visualization/visualization_function_macros.h"

namespace PMP = CGAL::Polygon_mesh_processing;
using namespace mgodpl;

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using CMesh = CGAL::Polyhedron_3<K>;
using SMesh = CGAL::Surface_mesh<K::Point_3>;
typedef CGAL::Mean_curvature_flow_skeletonization<CMesh> Skeletonization;
typedef Skeletonization::Skeleton Skeleton;

typedef Skeleton::vertex_descriptor Skeleton_vertex;
typedef Skeleton::edge_descriptor Skeleton_edge;

/**
 * Converts a custom Mesh object to a CGAL Surface_mesh.
 *
 * @param mesh The custom Mesh object representing the trunk.
 * @return A CGAL Surface_mesh object.
 */
SMesh convert_to_cgal_surface_mesh(const Mesh &mesh) {
	// It's pretty much a 1-to-1 conversion:
	SMesh cgal_mesh;
	for (const auto &vertex: mesh.vertices) {
		cgal_mesh.add_vertex({vertex.x(), vertex.y(), vertex.z()});
	}
	for (const auto &face: mesh.triangles) {
		cgal_mesh.add_face(SMesh::Vertex_index(face[0]), SMesh::Vertex_index(face[1]), SMesh::Vertex_index(face[2]));
	}
	return cgal_mesh;
}

/**
 * Converts a CGAL Surface_mesh to a custom Mesh object.
 *
 * @param cgal_mesh The CGAL Surface_mesh object.
 * @return A custom Mesh object representing the trunk.
 */
Mesh convert_from_cgal_surface_mesh(const SMesh &cgal_mesh) {
	// Allocate a new Mesh object:
	Mesh trunk_mesh;

	// Copy the vertices 1-to-1, converting them to our custom Vec3d type:
	trunk_mesh.vertices.reserve(cgal_mesh.number_of_vertices());
	for (const auto &vertex: cgal_mesh.vertices()) {
		trunk_mesh.vertices.push_back({
			cgal_mesh.point(vertex).x(), cgal_mesh.point(vertex).y(), cgal_mesh.point(vertex).z()
		});
	}

	// Also copy trinagles 1-to-1:
	trunk_mesh.triangles.reserve(cgal_mesh.number_of_faces());
	for (const auto &face: cgal_mesh.faces()) {
		// We need to follow the halfedges around the face to get the vertices. We assume all faces are triangles.
		auto [vitr,vend] = cgal_mesh.vertices_around_face(cgal_mesh.halfedge(face));
		auto v1 = *vitr++;
		auto v2 = *vitr++;
		auto v3 = *vitr;

		// Add the face:
		trunk_mesh.triangles.push_back({v1.idx(), v2.idx(), v3.idx()});
	}

	// Return the new Mesh object:
	return trunk_mesh;
}

using vertex_descriptor = boost::graph_traits<CMesh>::vertex_descriptor;
using halfedge_descriptor = boost::graph_traits<CMesh>::halfedge_descriptor;
using face_descriptor = boost::graph_traits<CMesh>::face_descriptor;

/**
 * Find holes in the polyhedron and close them by triangulating them.
 *
 * @param polyhedron The polyhedron in which to close holes; will be modified in-place.
 */
void close_holes(CMesh &polyhedron) {
	// Find holes in the polyhedron:
	std::vector<halfedge_descriptor> border_cycles;
	PMP::extract_boundary_cycles(polyhedron, std::back_inserter(border_cycles));

	// For each, close the hole by triangulating it:
	for (const auto h: border_cycles) {
		PMP::triangulate_hole(polyhedron, h);
	}
}


/**`
 * Some (most) of our tree meshes consist of several connected components, moved into each other
 * to give the appearance of a single tree. This function makes it so that the tree trunk is actually
 * a single mesh.
 *
 * If the mesh (polyhedron) is not outward oriented, it will be reversed.
 *
 * @param polyhedron The polyhedron to process; will be modified in-place.
 */
void separate_and_union_components(CMesh &polyhedron) {
	// Make sure the polyhedron is outward oriented, otherwise we end up doing intersection instead of union.
	if (!PMP::is_outward_oriented(polyhedron)) {
		PMP::reverse_face_orientations(polyhedron);
	}

	// Separate connected components
	std::vector<CMesh> components;
	PMP::split_connected_components(polyhedron, components);

	// Compute the union of all components
	CMesh &union_mesh = components[0];
	for (std::size_t i = 1; i < components.size(); ++i) {
		// Docs: "If tm_out is one of the input surface meshes, it will be updated to contain the output (in-place operation)[...]"
		bool success = PMP::corefine_and_compute_union(union_mesh, components[i], union_mesh);
		if (!success) {
			std::cerr << "Union of components " << i << " and " << i + 1 << " failed." << std::endl;
		}
	}

	// Update the polyhedron with the union mesh:
	polyhedron = union_mesh;
}

/**
 * Finds the vertex in the skeleton that is closest to the origin (0, 0, 0).
 *
 * @param skeleton The skeleton to search.
 * @return The vertex descriptor of the closest vertex.
 */
Skeleton_vertex find_lowest_point(const Skeleton &skeleton) {
	// This is just a simple linear search:
	double min_distance = std::numeric_limits<double>::max();
	Skeleton_vertex closest_vertex = 0;
	auto [start, end] = vertices(skeleton);
	for (auto itr = start; itr != end; ++itr) {
		double distance = skeleton[*itr].point.x() * skeleton[*itr].point.x() +
		                  skeleton[*itr].point.y() * skeleton[*itr].point.y() +
		                  skeleton[*itr].point.z() * skeleton[*itr].point.z();
		if (distance < min_distance) {
			closest_vertex = itr - start;
			min_distance = distance;
		}
	}
	return closest_vertex;
}

/**
 * Performs Dijkstra's algorithm on the given skeleton to find the shortest paths
 * from the given root vertex to all other vertices.
 *
 * @param skeleton The skeleton graph.
 * @param root The vertex to start the Dijkstra's algorithm from.
 * @return A pair containing the distances and predecessors vectors; indexed by vertex descriptor of the skeleton.
 */
std::pair<std::vector<double>, std::vector<Skeleton_vertex> > dijkstra_skeleton(
	const Skeleton &skeleton,
	Skeleton_vertex root) {
	// Prepare the vectors to store the results:
	std::vector<double> distances(num_vertices(skeleton));
	std::vector<Skeleton_vertex> predecessors(num_vertices(skeleton));

	// Create a weight map for the edges, based on a lambda function
	// that calculates the Euclidean distance between the endpoints of the edge.
	auto weight_map = boost::make_function_property_map<Skeleton::edge_descriptor>(
		[&skeleton](const Skeleton::edge_descriptor &e) -> double {
			auto v1 = skeleton[source(e, skeleton)].point;
			auto v2 = skeleton[target(e, skeleton)].point;
			return sqrt((v1 - v2).squared_length());
		});

	// Run Dijkstra's algorithm:
	boost::dijkstra_shortest_paths(skeleton,
	                               root,
	                               boost::weight_map(weight_map)
	                               .distance_map(
		                               boost::make_iterator_property_map(
			                               distances.begin(),
			                               get(boost::vertex_index, skeleton)))
	                               .predecessor_map(
		                               boost::make_iterator_property_map(
			                               predecessors.begin(),
			                               get(boost::vertex_index, skeleton))));

	// Return the results:
	return {distances, predecessors};
}

/**
 * Creates a vector of vertex descriptors that represents a hierarchical order of the skeleton vertices;
 * i.e. later vertices are guaranteed to appear after ones closer to the root in the shortest path tree.
 *
 * The first vertex in the returned vector is the root vertex.
 *
 * @param skeleton		The skeleton graph.
 * @param distances	    The Dijkstra distances from the root vertex, see `dijkstra_skeleton`.
 *
 * @return A vector of vertex descriptors representing the hierarchical order.
 */
std::vector<Skeleton::vertex_descriptor> create_hierarchical_order(const Skeleton &skeleton,
                                                                   const std::vector<double> &distances) {
	// Create a vector to store the hierarchical order:
	std::vector<Skeleton::vertex_descriptor> hierarchical_order;
	hierarchical_order.reserve(num_vertices(skeleton));

	// Fill with indices 0 to n-1:
	for (std::size_t i = 0; i < num_vertices(skeleton); ++i) {
		hierarchical_order.push_back(i);
	}

	// Sort by the dijkstra distance, using the vertex descriptor to index the distances vector:
	std::ranges::sort(hierarchical_order,
	                  [&distances](Skeleton::vertex_descriptor a, Skeleton::vertex_descriptor b) {
		                  return distances[a] < distances[b];
	                  });

	// Return the sorted vector:
	return hierarchical_order;
}

/// Converts a CGAL Point_3 to our custom Vec3d type.
math::Vec3d toVec3d(const K::Point_3 &point) {
	return {point.x(), point.y(), point.z()};
}

/**
 * Creates a ground plane mesh of the specified size.
 *
 * @param size The size of the ground plane.
 * @return A Mesh object representing the ground plane.
 */
mgodpl::Mesh ground_plane_2(double size) {
	mgodpl::Mesh ground;
	ground.vertices = {
		{-size, -size, 0},
		{size, -size, 0},
		{size, size, 0},
		{-size, size, 0}
	};
	ground.triangles = {
		{0, 1, 2},
		{0, 2, 3}
	};
	return ground;
}

/**
 * Finds the closest vertex in the skeleton to the given vertex.
 *
 * @param vertex The vertex to find the closest skeleton vertex to.
 * @param skeleton The skeleton graph.
 * @return The vertex descriptor of the closest vertex in the skeleton.
 */
Skeleton::vertex_descriptor find_closest_vertex(const math::Vec3d &vertex, const Skeleton &skeleton) {
	double min_distance = std::numeric_limits<double>::max();
	Skeleton::vertex_descriptor closest_vertex = 0;
	for (std::size_t i = 0; i < num_vertices(skeleton); ++i) {
		math::Vec3d skeleton_vertex(skeleton[i].point.x(), skeleton[i].point.y(), skeleton[i].point.z());
		double distance = (vertex - skeleton_vertex).norm();
		if (distance < min_distance) {
			min_distance = distance;
			closest_vertex = i;
		}
	}
	return closest_vertex;
}

/**
 * For every vertex in the given mesh, find the closest vertex in the skeleton.
 *
 * @param mesh The mesh that we'd like to find the closest skeleton vertices for.
 * @param skeleton The skeleton graph.
 * @return A vector of, for each vertex in the mesh, the vertex descriptor of the closest vertex in the skeleton.
 */
std::vector<Skeleton::vertex_descriptor> find_closest_skeleton_vertices(
	const Mesh &mesh,
	const Skeleton &skeleton) {
	// Create a vector to store the closest skeleton vertex for each vertex in the mesh:
	std::vector<Skeleton::vertex_descriptor> skeleton_attachment;
	skeleton_attachment.reserve(mesh.vertices.size());

	for (const auto &vertex: mesh.vertices) {
		skeleton_attachment.push_back(find_closest_vertex(vertex, skeleton));
	}
	return skeleton_attachment;
}

/**
 * Cleans up the trunk mesh by closing holes and unifying connected components.
 *
 * The trunk meshes we have are a bit messy, made to look good visually, but with poor mesh topology,
 * featuring open ends at the end of branches, or models kinda pushed into each other.
 *
 * This function cleans up the mesh by closing holes and unifying connected components.
 *
 * @param polyhedron The polyhedron representing the trunk mesh to be cleaned up; will be modified in-place.
 */
void clean_up_trunk_mesh(CMesh &polyhedron) {
	// Close holes:
	close_holes(polyhedron);

	// Separate and union components:
	separate_and_union_components(polyhedron);
}

/**
 * Deforms the given mesh based on the skeleton and the new points.
 *
 * Applies a translation to every vertex in the mesh, such that the vertex is moved the same amount
 * as the associated vertex in the skeleton.
 *
 * @param mesh			The mesh to deform.
 * @param skeleton		The skeleton of the trunk mesh, without deformation.
 * @param skeleton_attachment A vector that associates each vertex in the mesh with a vertex in the skeleton.
 * @param new_points    The position of the skeleton vertices after deformation.
 * @return The deformed mesh.
 */
Mesh deform_mesh(const Mesh &mesh,
                 const Skeleton &skeleton,
                 const std::vector<Skeleton::vertex_descriptor> &skeleton_attachment,
                 const std::vector<math::Vec3d> &new_points) {
	// Create a copy of the mesh to deform:
	Mesh deformed_mesh = mesh;

	// For each vertex in the mesh, calculate the new position based on the skeleton attachment:
	for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
		deformed_mesh.vertices[i] = mesh.vertices[i] + (new_points[skeleton_attachment[i]] - toVec3d(
			                                                skeleton[skeleton_attachment[i]].point));
	}

	return deformed_mesh;
}

REGISTER_VISUALIZATION(tree_skeleton_animation) {
	const auto meshes = tree_meshes::loadTreeMeshes("appletree1");
	// Load the apple tree trunk model:
	const auto trunk = meshes.trunk_mesh;

	// Convert the tree trunk mesh to a CGAL surface mesh:
	SMesh cgal_mesh = convert_to_cgal_surface_mesh(trunk);

	// Convert to a polyhedron:
	CMesh polyhedron;
	CGAL::copy_face_graph(cgal_mesh, polyhedron);

	// Clean up the trunk mesh
	clean_up_trunk_mesh(polyhedron);

	Skeletonization mcs(polyhedron);

	// Iteratively apply step 1 to 3 until convergence.
	mcs.contract_until_convergence();

	// Convert the contracted mesh into a curve skeleton and
	Skeleton skeleton;

	// get the correspondent surface points
	mcs.convert_to_skeleton(skeleton);

	// Find the point closest to 0:
	Skeleton_vertex skeleton_root = find_lowest_point(skeleton);

	// Perform Dijkstra's algorithm on the skeleton to find the shortest paths from the closest vertex
	const auto &[distances, predecessors] = dijkstra_skeleton(skeleton, skeleton_root);

	// Find the maximum Dijkstra distance
	double max_dijkstra_distance = *std::ranges::max_element(distances);

	// Create a hierarchical order of the skeleton vertices based on the Dijkstra distances
	auto hierarchical_order = create_hierarchical_order(skeleton, distances);

	VtkLineSegmentsVisualization skeleton_viz(1, 0, 0);
	// viewer.addActor(skeleton_viz.getActor());

	// Convert it back from the polyhedron:
	SMesh cgal_mesh_back;
	CGAL::copy_face_graph(polyhedron, cgal_mesh_back);

	Mesh trunk_mesh = convert_from_cgal_surface_mesh(cgal_mesh_back);

	auto skeleton_attachment = find_closest_skeleton_vertices(trunk_mesh, skeleton);
	auto leaves_skeleton_attachment = find_closest_skeleton_vertices(meshes.leaves_mesh, skeleton);

	viewer.lockCameraUp();

	viewer.addMesh(ground_plane_2(10), {0.4, 0.8, 0.4}, 1.0);

	viewer.setCameraTransform({3.0, 4.0, 2.0}, {0.0, 0.0, 1.5});

	std::optional<vtkActor *> deformed_trunk_mesh_actor;
	std::optional<vtkActor *> deformed_leaves_mesh_actor;

	double t = 0.0;
	viewer.addTimerCallback([&] {
		t += 0.1;

		std::vector<math::Vec3d> new_points(num_vertices(skeleton));
		new_points[hierarchical_order[0]] = toVec3d(skeleton.m_vertices[hierarchical_order[0]].m_property.point);
		std::vector<math::Quaterniond> rotations(num_vertices(skeleton));
		rotations[0] = math::Quaterniond::fromAxisAngle({1, 0, 0}, 0);
		for (const std::size_t i: std::ranges::subrange(hierarchical_order.begin() + 1, hierarchical_order.end())) {
			double r = sin(t) * sin(t * 0.74) * (0.5 * 0.5 * sin(t * 0.1));
			r *= distances[i] / max_dijkstra_distance;
			rotations[i] = math::Quaterniond::fromAxisAngle({1, 0, 0}, r);
		}

		new_points[hierarchical_order[0]] = toVec3d(skeleton.m_vertices[hierarchical_order[0]].m_property.point);
		for (std::size_t i: std::ranges::subrange(hierarchical_order.begin() + 1, hierarchical_order.end())) {
			new_points[i] = new_points[predecessors[i]] + rotations[i].rotate(
				                toVec3d(skeleton[i].point) - toVec3d(skeleton[predecessors[i]].point));
		}

		std::vector<std::pair<math::Vec3d, math::Vec3d> > lines;
		for (const auto &edge: skeleton.m_edges) {
			auto v1 = new_points[source(edge, skeleton)];
			auto v2 = new_points[target(edge, skeleton)];
			lines.push_back(std::make_pair(v1, v2));
		}

		skeleton_viz.updateLine(lines); {
			Mesh trunk_mesh_deformed = deform_mesh(trunk_mesh, skeleton, skeleton_attachment, new_points);

			if (deformed_trunk_mesh_actor) {
				viewer.removeActor(*deformed_trunk_mesh_actor);
			}
			deformed_trunk_mesh_actor = viewer.addMesh(trunk_mesh_deformed, WOOD_COLOR, 1.0);
		} {
			Mesh trunk_mesh_deformed = meshes.leaves_mesh;
			for (std::size_t i = 0; i < meshes.leaves_mesh.vertices.size(); ++i) {
				math::Vec3d trunk_vertex = meshes.leaves_mesh.vertices[i];
				math::Vec3d skeleton_vertex = toVec3d(skeleton[leaves_skeleton_attachment[i]].point);

				math::Vec3d delta = skeleton_vertex - trunk_vertex;
				trunk_mesh_deformed.vertices[i] = new_points[leaves_skeleton_attachment[i]] - delta;
			}

			if (deformed_leaves_mesh_actor) {
				viewer.removeActor(*deformed_leaves_mesh_actor);
			}
			deformed_leaves_mesh_actor = viewer.addMesh(trunk_mesh_deformed, LEAF_COLOR, 1.0);
		}

		if (t > 30 && viewer.isRecording()) {
			viewer.stop();
		}
	});

	viewer.start();
}
