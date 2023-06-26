// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 14-6-23.
//

#ifndef MGODPL_CGALMESHSHELL_H
#define MGODPL_CGALMESHSHELL_H


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_shortest_path/Surface_mesh_shortest_path_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/convex_hull_3.h>

namespace mgodpl {

	struct CGALMeshShell {

		using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
		using Triangle_mesh = CGAL::Surface_mesh<Kernel::Point_3>;
		using Traits = CGAL::Surface_mesh_shortest_path_traits<Kernel, Triangle_mesh>;
		using Primitive = CGAL::AABB_face_graph_triangle_primitive<Triangle_mesh>;
		using AABBTraits = CGAL::AABB_traits<Kernel, Primitive>;
		using Surface_mesh_shortest_path = CGAL::Surface_mesh_shortest_path<Traits>;
		using CGALMeshPoint = Surface_mesh_shortest_path::Face_location;

		struct ShellPoint {
			CGALMeshPoint point;
			/// We use an explicit normal here, because the surface normal is poorly defined at the edges and vertices.
			Eigen::Vector3d normal;
		};

		/// The CGAL mesh (a halfedge datastructure) for topology-aware shortest-paths computation.
		Triangle_mesh tmesh;

		/// An AABB-tree for quick lookup of the on_which_mesh point on the mesh (including facet information)
		CGAL::AABB_tree <AABBTraits> tree{};

		/// When computing the path_on_shell, the states will be offset from the shell by this distance.
		double padding = 0.1;

		/// How heavy to weigh rotation in predict_path_length.
		double rotation_weight = 1.0;

		CGALMeshShell(Triangle_mesh mesh, double rotationWeight, double padding)
				: rotation_weight(rotationWeight), padding(padding),tmesh(mesh) {

			// We initialize the AABB tree such that we don't have to re-compute it every projection query.
			Surface_mesh_shortest_path shortest_paths(tmesh);
			shortest_paths.build_aabb_tree(tree);

		}

	};

	namespace distance_matrix {

		template<>
		double point_distance(const CGALMeshShell &shell, const CGALMeshShell::ShellPoint &a, const CGALMeshShell::ShellPoint &b) {

		}

	}

}


#endif //MGODPL_CGALMESHSHELL_H
