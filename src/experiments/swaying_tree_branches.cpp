// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

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


REGISTER_VISUALIZATION(tree_skeletonization) {
	// Load the apple tree trunk model:
	auto trunk = tree_meshes::loadTreeMeshes("appletree").trunk_mesh;

	// Convert the tree trunk mesh to a CGAL surface mesh:
	SMesh cgal_mesh = convert_to_cgal_surface_mesh(trunk);

	// Convert to a polyhedron:
	CMesh polyhedron;
	CGAL::copy_face_graph(cgal_mesh, polyhedron);

	// Close holes:
	// collect one halfedge per boundary cycle
	close_holes(polyhedron);


	Skeleton skeleton;
	Skeletonization mcs(polyhedron);

	// Iteratively apply step 1 to 3 until convergence.
	mcs.contract_until_convergence();


	// Convert the contracted mesh into a curve skeleton and
	// get the correspondent surface points
	mcs.convert_to_skeleton(skeleton);

	VtkLineSegmentsVisualization skeleton_viz(1, 0, 0);
	viewer.addActor(skeleton_viz.getActor());

	std::vector<std::pair<math::Vec3d, math::Vec3d> > lines;
	for (const auto &edge: skeleton.m_edges) {
		auto v1 = skeleton[source(edge, skeleton)].point;
		auto v2 = skeleton[target(edge, skeleton)].point;

		auto pair = std::make_pair(math::Vec3d(v1.x(), v1.y(), v1.z()), math::Vec3d(v2.x(), v2.y(), v2.z()));
		lines.push_back(pair);
	}
	skeleton_viz.updateLine(lines);

	// Convert it back from the polyhedron:
	SMesh cgal_mesh_back;
	CGAL::copy_face_graph(polyhedron, cgal_mesh_back);

	Mesh trunk_mesh = convert_from_cgal_surface_mesh(cgal_mesh_back);

	viewer.addMesh(trunk_mesh, {0.5, 0.5, 0.5}, 0.1);

	viewer.start();
}
