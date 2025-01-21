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

namespace PMP = CGAL::Polygon_mesh_processing;
#include "../visualization/visualization_function_macros.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../visualization/VtkLineSegmentVizualization.h"

using namespace mgodpl;

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using CMesh = CGAL::Polyhedron_3<K>;
using SMesh = CGAL::Surface_mesh<K::Point_3>;
typedef CGAL::Mean_curvature_flow_skeletonization<CMesh> Skeletonization;
typedef Skeletonization::Skeleton Skeleton;

typedef Skeleton::vertex_descriptor Skeleton_vertex;
typedef Skeleton::edge_descriptor Skeleton_edge;

SMesh convert_to_cgal_surface_mesh(const Mesh &trunk) {
	SMesh cgal_mesh;
	for (const auto &vertex: trunk.vertices) {
		cgal_mesh.add_vertex({vertex.x(), vertex.y(), vertex.z()});
	}
	for (const auto &face: trunk.triangles) {
		cgal_mesh.add_face(SMesh::Vertex_index(face[0]), SMesh::Vertex_index(face[1]), SMesh::Vertex_index(face[2]));
	}
	return cgal_mesh;
}

Mesh convert_from_cgal_surface_mesh(const SMesh &cgal_mesh) {
	Mesh trunk_mesh;
	for (const auto &vertex: cgal_mesh.vertices()) {
		trunk_mesh.vertices.push_back({
			cgal_mesh.point(vertex).x(), cgal_mesh.point(vertex).y(), cgal_mesh.point(vertex).z()
		});
	}
	for (const auto &face: cgal_mesh.faces()) {
		auto [vitr,vend] = cgal_mesh.vertices_around_face(cgal_mesh.halfedge(face));
		auto v1 = *vitr++;
		auto v2 = *vitr++;
		auto v3 = *vitr;

		// Add the face:
		trunk_mesh.triangles.push_back({v1.idx(), v2.idx(), v3.idx()});
	}
	return trunk_mesh;
}

using vertex_descriptor = boost::graph_traits<CMesh>::vertex_descriptor;
using halfedge_descriptor = boost::graph_traits<CMesh>::halfedge_descriptor;
using face_descriptor = boost::graph_traits<CMesh>::face_descriptor;

void close_holes(CMesh &polyhedron) {
	std::vector<halfedge_descriptor> border_cycles;
	PMP::extract_boundary_cycles(polyhedron, std::back_inserter(border_cycles));
	for (const auto h: border_cycles) {
		PMP::triangulate_hole(polyhedron, h);
	}
}

#include <CGAL/Polygon_mesh_processing/orientation.h>

void separate_and_union_components(CMesh &polyhedron) {
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

	polyhedron = union_mesh;
}

Skeleton_vertex find_lowest_point(const Skeleton &skeleton) {
	double min_distance = std::numeric_limits<double>::max();
	Skeleton_vertex closest_vertex;
	auto [start, end] = vertices(skeleton);
	for (auto itr = start; itr != end; ++itr) {
		double distance = skeleton[*itr].point.x() * skeleton[*itr].point.x() + skeleton[*itr].point.y() * skeleton[*
			                  itr].point.y() + skeleton[*itr].point.z() * skeleton[*itr].point.z();
		if (distance < min_distance) {
			closest_vertex = itr - start;
			min_distance = distance;
		}
	}
	return closest_vertex;
}

// struct edge_weight_map {
// 	using edge_descriptor = Skeleton::edge_descriptor;
// 	using value_type = double;
// 	using reference = double;
// 	using key_type = edge_descriptor;
// 	using category = boost::readable_property_map_tag;
//
// 	edge_weight_map(const Skeleton &skeleton) : skeleton(skeleton) {
// 	}
//
// 	double operator[](const edge_descriptor &e) const {
// 		auto v1 = skeleton[source(e, skeleton)].point;
// 		auto v2 = skeleton[target(e, skeleton)].point;
// 		return std::sqrt(std::pow(v1.x() - v2.x(), 2) + std::pow(v1.y() - v2.y(), 2) + std::pow(v1.z() - v2.z(), 2));
// 	}
//
// 	const Skeleton &skeleton;
// };

std::pair<std::vector<double>, std::vector<Skeleton_vertex> > dijkstra_skeleton(
	const Skeleton &skeleton,
	Skeleton_vertex closest_vertex) {
	std::vector<double> distances(num_vertices(skeleton));
	std::vector<Skeleton_vertex> predecessors(num_vertices(skeleton));

	auto weight_map = boost::make_function_property_map<Skeleton::edge_descriptor>(
		[&skeleton](const Skeleton::edge_descriptor &e) -> double {
			auto v1 = skeleton[source(e, skeleton)].point;
			auto v2 = skeleton[target(e, skeleton)].point;
			return sqrt((v1 - v2).squared_length());
		});

	boost::dijkstra_shortest_paths(skeleton,
	                               closest_vertex,
	                               boost::weight_map(weight_map)
	                               .distance_map(
		                               boost::make_iterator_property_map(
			                               distances.begin(),
			                               get(boost::vertex_index, skeleton)))
	                               .predecessor_map(
		                               boost::make_iterator_property_map(
			                               predecessors.begin(),
			                               get(boost::vertex_index, skeleton))));
	return {distances, predecessors};
}

std::vector<std::pair<math::Vec3d, math::Vec3d> > extract_lines_from_skeleton(const Skeleton &skeleton) {
	std::vector<std::pair<math::Vec3d, math::Vec3d> > lines;
	for (const auto &edge: skeleton.m_edges) {
		auto v1 = skeleton[source(edge, skeleton)].point;
		auto v2 = skeleton[target(edge, skeleton)].point;

		auto pair = std::make_pair(math::Vec3d(v1.x(), v1.y(), v1.z()), math::Vec3d(v2.x(), v2.y(), v2.z()));
		lines.push_back(pair);
	}
	return lines;
}

math::Vec3d project_onto_skeleton_edge(const Skeleton &skeleton,
                                       Skeleton::edge_descriptor edge,
                                       const math::Vec3d &vertex) {
	auto v1 = skeleton[source(edge, skeleton)].point;
	auto v2 = skeleton[target(edge, skeleton)].point;

	math::Vec3d p1(v1.x(), v1.y(), v1.z());
	math::Vec3d p2(v2.x(), v2.y(), v2.z());
	math::Vec3d p3(vertex.x(), vertex.y(), vertex.z());

	double a = (p2 - p1).norm();
	double b = (p3 - p1).norm();

	math::Vec3d projection = p1 + (p2 - p1) * (b / a);
	return projection;
}

Skeleton::edge_descriptor find_closest_edge_on_skeleton(const Skeleton &skeleton, const math::Vec3d &vertex) {
	Skeleton::edge_descriptor closest_edge;
	double min_distance = std::numeric_limits<double>::max();

	const auto [start, end] = edges(skeleton);
	for (auto itr = start; itr != end; ++itr) {
		math::Vec3d p3(vertex.x(), vertex.y(), vertex.z());
		math::Vec3d projection = project_onto_skeleton_edge(skeleton, *itr, vertex);

		double distance = (p3 - projection).norm();

		if (distance < min_distance) {
			min_distance = distance;
			closest_edge = *itr;
		}
	}

	return closest_edge;
}

double find_dijkstra_distance(const Skeleton &skeleton,
                              const math::Vec3d &vertex,
                              const std::vector<double> &distances) {
	Skeleton::edge_descriptor closest_edge = find_closest_edge_on_skeleton(skeleton, vertex);

	double source_distance = distances[source(closest_edge, skeleton)];
	double target_distance = distances[target(closest_edge, skeleton)];

	// Find the closest point on the edge:
	math::Vec3d projection = project_onto_skeleton_edge(skeleton, closest_edge, vertex);

	double distance_from_source = (projection - math::Vec3d(
		                               skeleton[source(closest_edge, skeleton)].point.x(),
		                               skeleton[source(closest_edge, skeleton)].point.y(),
		                               skeleton[source(closest_edge, skeleton)].point.z())).norm();

	double distance_from_target = (projection - math::Vec3d(
		                               skeleton[target(closest_edge, skeleton)].point.x(),
		                               skeleton[target(closest_edge, skeleton)].point.y(),
		                               skeleton[target(closest_edge, skeleton)].point.z())).norm();

	double dijkstra_distance_1 = distance_from_source + source_distance;
	double dijkstra_distance_2 = distance_from_target + target_distance;

	return std::min(dijkstra_distance_1, dijkstra_distance_2);
}

std::vector<Skeleton::vertex_descriptor> create_hierarchical_order(const Skeleton &skeleton,
                                                                   const std::vector<double> &distances) {
	std::vector<Skeleton::vertex_descriptor> hierarchical_order;
	hierarchical_order.reserve(num_vertices(skeleton));
	// Fill with indices 0 to n-1:
	for (std::size_t i = 0; i < num_vertices(skeleton); ++i) {
		hierarchical_order.push_back(i);
	}
	// Sort by the dijkstra distance:
	std::ranges::sort(hierarchical_order,
	                  [&distances](Skeleton::vertex_descriptor a, Skeleton::vertex_descriptor b) {
		                  return distances[a] < distances[b];
	                  });
	return hierarchical_order;
}

std::vector<math::Vec3d> compute_deltas(const Skeleton &skeleton,
                                        const std::vector<Skeleton_vertex> &predecessors) {
	std::vector<math::Vec3d> predecessor_deltas;
	predecessor_deltas.reserve(num_vertices(skeleton));
	for (std::size_t i = 0; i < num_vertices(skeleton); ++i) {
		Skeleton_vertex predecessor = predecessors[i];
		if (predecessor == i) {
			predecessor_deltas.push_back({0, 0, 0});
		} else {
			math::Vec3d predecessor_point(skeleton[predecessor].point.x(),
			                              skeleton[predecessor].point.y(),
			                              skeleton[predecessor].point.z());
			math::Vec3d current_point(skeleton[i].point.x(),
			                          skeleton[i].point.y(),
			                          skeleton[i].point.z());
			predecessor_deltas.push_back(current_point - predecessor_point);
		}
	}
	return predecessor_deltas;
}

math::Vec3d toVec3d(const K::Point_3 &point) {
	return {point.x(), point.y(), point.z()};
}

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


REGISTER_VISUALIZATION(tree_skeletonization) {
	// Load the apple tree trunk model:
	auto trunk = tree_meshes::loadTreeMeshes("appletree1").trunk_mesh;

	// Convert the tree trunk mesh to a CGAL surface mesh:
	SMesh cgal_mesh = convert_to_cgal_surface_mesh(trunk);

	// Convert to a polyhedron:
	CMesh polyhedron;
	CGAL::copy_face_graph(cgal_mesh, polyhedron);

	// Close holes:
	// collect one halfedge per boundary cycle
	close_holes(polyhedron);

	separate_and_union_components(polyhedron);

	Skeletonization mcs(polyhedron);

	// Iteratively apply step 1 to 3 until convergence.
	mcs.contract_until_convergence();

	// Convert the contracted mesh into a curve skeleton and
	Skeleton skeleton;

	// get the correspondent surface points
	mcs.convert_to_skeleton(skeleton);

	// Find the point closest to 0:
	Skeleton_vertex closest_vertex = find_lowest_point(skeleton);

	// Perform Dijkstra's algorithm on the skeleton to find the shortest paths from the closest vertex
	const auto &[distances, predecessors] = dijkstra_skeleton(skeleton, closest_vertex);

	// Find the maximum Dijkstra distance
	double max_dijkstra_distance = *std::ranges::max_element(distances);

	// Create a hierarchical order of the skeleton vertices based on the Dijkstra distances
	auto hierarchical_order = create_hierarchical_order(skeleton, distances);

	// Compute the deltas between each vertex and its predecessor in the shortest path tree
	auto rest_deltas = compute_deltas(skeleton, predecessors);


	VtkLineSegmentsVisualization skeleton_viz(1, 0, 0);
	viewer.addActor(skeleton_viz.getActor());


	// // Convert it back from the polyhedron:
	// SMesh cgal_mesh_back;
	// CGAL::copy_face_graph(polyhedron, cgal_mesh_back);
	//
	// Mesh trunk_mesh = convert_from_cgal_surface_mesh(cgal_mesh_back);
	//
	// // For each vertex in the mesh, find the Dijkstra distance to the lowest point:
	// std::vector<double> dijkstra_distances;
	// dijkstra_distances.reserve(trunk_mesh.vertices.size());
	// for (const auto &vertex: trunk_mesh.vertices) {
	// 	double dijkstra_distance = find_dijkstra_distance(skeleton, vertex, distances);
	// 	dijkstra_distances.push_back(dijkstra_distance);
	// }
	// double max_dijkstra_distance = *std::ranges::max_element(dijkstra_distances);

	// // At every vertex, add a sphere with radius proportional to dijkstra_distance / max_dijkstra_distance:
	// for (std::size_t i = 0; i < trunk_mesh.vertices.size(); ++i) {
	// 	viewer.addSphere(0.1 * dijkstra_distances[i] / max_dijkstra_distance, trunk_mesh.vertices[i], {0, 1, 0}, 4);
	// }

	// Mesh trunk_mesh_deformed = trunk_mesh;
	// for (std::size_t i = 0; i < trunk_mesh.vertices.size(); ++i) {
	// 	trunk_mesh_deformed.vertices[i] = trunk_mesh.vertices[i] + math::Vec3d(
	// 		                                  0,
	// 		                                  0,
	// 		                                  dijkstra_distances[i] / max_dijkstra_distance);
	// }

	// viewer.addMesh(trunk_mesh, {0.5, 0.5, 0.5}, 0.5);
	// viewer.addMesh(trunk_mesh_deformed, {1.0, 0.0, 1.0}, 0.5);

	viewer.lockCameraUp();

	viewer.addMesh(ground_plane_2(10), {0.8, 0.8, 0.8}, 1.0);

	viewer.setCameraTransform({1.0, 5.0, 2.5}, {0.0, 0.0, 1.5});

	double t = 0.0;
	viewer.addTimerCallback([&] {
		t += 0.1;
		math::Vec3d wind_direction(sin(t) * sin(t * 0.74) * (0.5 * 0.5 * sin(t * 0.1)), 0, 0);

		std::vector<math::Vec3d> new_deltas;
		new_deltas.reserve(num_vertices(skeleton));
		// Ajust the deltas:
		for (std::size_t i = 0; i < num_vertices(skeleton); ++i) {
			double delta_length = rest_deltas[i].norm();
			math::Vec3d delta = rest_deltas[i];
			math::Vec3d new_delta = delta + wind_direction * delta_length * (distances[i] / max_dijkstra_distance);
			// adjust the length:
			new_delta = new_delta / new_delta.norm() * delta_length;
			new_deltas.push_back(new_delta);
		}

		std::vector<math::Vec3d> new_points(num_vertices(skeleton));
		new_points[hierarchical_order[0]] = toVec3d(skeleton.m_vertices[hierarchical_order[0]].m_property.point);
		for (std::size_t i: std::ranges::subrange(hierarchical_order.begin() + 1, hierarchical_order.end())) {
			new_points[i] = new_points[predecessors[i]] + new_deltas[i];
		}

		// // At every skeleton vertex, put a sphere of radius proportional to the distance to the lowest point:
		// auto [start, end] = vertices(skeleton);
		// for (auto vertex = start; vertex != end; ++vertex) {
		// 	math::Vec3d point(skeleton[*vertex].point.x(), skeleton[*vertex].point.y(), skeleton[*vertex].point.z());
		// 	viewer.addSphere(0.1 * distances[*vertex] / max_dijkstra_distance, point, {0, 1, 0}, 4);
		// }

		std::vector<std::pair<math::Vec3d, math::Vec3d> > lines;
		for (const auto &edge: skeleton.m_edges) {
			auto v1 = new_points[source(edge, skeleton)];
			auto v2 = new_points[target(edge, skeleton)];
			lines.push_back(std::make_pair(v1, v2));
		}

		skeleton_viz.updateLine(lines);

		if (t > 10 && viewer.isRecording()) {
			viewer.stop();
		}
	});

	viewer.start();
}
