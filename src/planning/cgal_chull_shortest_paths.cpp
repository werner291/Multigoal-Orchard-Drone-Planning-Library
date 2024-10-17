// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 7-11-23.
//

#include <CGAL/convex_hull_3.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include "cgal_chull_shortest_paths.h"

namespace mgodpl::cgal {

	void PathVisitor::operator()(Surface_mesh_shortest_path::halfedge_descriptor edge, Surface_mesh_shortest_path::FT t) {
		states.push_back(path_algo.face_location(edge, t));
		states.push_back(path_algo.face_location(mesh.opposite(edge), 1.0 - t));
	}

	void PathVisitor::operator()(Surface_mesh_shortest_path::vertex_descriptor vertex) {
		states.push_back(path_algo.face_location(vertex));
	}

	void PathVisitor::operator()(Surface_mesh_shortest_path::face_descriptor f,
								 Surface_mesh_shortest_path::Barycentric_coordinates location) {
		states.push_back({f, location});
	}

	CgalMeshData::CgalMeshData(const Mesh &leaves_mesh)
			: convex_hull(cgal_convex_hull_around_leaves(leaves_mesh)),
			  mesh_path(convex_hull),
			  tree()
	{
		mesh_path.build_aabb_tree(tree);
	}

	mgodpl::cgal::Surface_mesh cgal_convex_hull_around_leaves(const Mesh &leaves_mesh) {
		Surface_mesh convex_hull;
		{
			std::vector<Point_3> cgal_points;
			for (const auto &point: leaves_mesh.vertices) {
				cgal_points.emplace_back(point.x(), point.y(), point.z());
			}
			CGAL::convex_hull_3(cgal_points.begin(), cgal_points.end(), convex_hull);
		}
		return convex_hull;
	}

	Surface_mesh_shortest_path::Face_location locate_nearest(const math::Vec3d &pt, const CgalMeshData &data) {
		return data.mesh_path.locate(to_cgal_point(pt), data.tree);
	}

	SurfacePointAndNormal from_face_location(const mgodpl::cgal::Surface_mesh_shortest_path::Face_location &fl, const CgalMeshData &data) {
		auto pt = data.mesh_path.point(fl.first, fl.second);
		auto normal = CGAL::Polygon_mesh_processing::compute_face_normal(fl.first, data.convex_hull);

		return {
				{pt.x(),pt.y(),pt.z()},
				{normal.x(),normal.y(),normal.z()},
		};
	}

	Mesh cgalMeshToMesh(const cgal::Surface_mesh &cgal_mesh) {
		Mesh mesh;

		// Convert the CGAL mesh to a Mesh:
		mesh.vertices.reserve(cgal_mesh.number_of_vertices());

		// First, convert the vertices:
		for (const auto &vertex: cgal_mesh.vertices()) {
			mesh.vertices.emplace_back(
				cgal_mesh.point(vertex).x(),
				cgal_mesh.point(vertex).y(),
				cgal_mesh.point(vertex).z()
			);
		}

		mesh.triangles.reserve(cgal_mesh.number_of_faces());

		// Then the triangles:
		for (const auto &f: cgal_mesh.faces()) {
			auto around_face = cgal_mesh.vertices_around_face(cgal_mesh.halfedge(f));

			// Make sure it's exactly 3 long:
			assert(around_face.size() == 3);

			auto iter = around_face.begin();

			auto v1 = *iter++;
			auto v2 = *iter++;
			auto v3 = *iter;

			mesh.triangles.push_back({v1, v2, v3});
		}

		return mesh;
	}
}
