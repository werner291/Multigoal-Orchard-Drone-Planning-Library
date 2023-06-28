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

#include "../distance_matrix.h"
#include "../../utilities/cgal_utils.h"
#include "../Shell.h"

namespace mgodpl {

	namespace mesh_shell {

		using namespace cgal_utils;

		struct CGALMeshShell {

			/// The CGAL mesh (a halfedge datastructure) for topology-aware shortest-paths computation.
			Triangle_mesh tmesh;

			/// An AABB-tree for quick lookup of the on_which_mesh point on the mesh (including facet information)
			CGAL::AABB_tree<AABBTraits> tree{};

			/// When computing the path_on_shell, the states will be offset from the shell by this distance.
			double padding = 0.1;

			/// How heavy to weigh rotation in predict_path_length.
			double rotation_weight = 1.0;

			CGALMeshShell(Triangle_mesh mesh, double rotationWeight, double padding)
					: rotation_weight(rotationWeight), padding(padding), tmesh(mesh) {

				// We initialize the AABB tree such that we don't have to re-compute it every projection query.
				Surface_mesh_shortest_path shortest_paths(tmesh);
				shortest_paths.build_aabb_tree(tree);

			}

		};

	}

	template<>
	struct shell_point_t<mesh_shell::CGALMeshShell> {
		using type = cgal_utils::CGALMeshPointAndNormal;
	};

	template<>
	struct shell_path_t<mesh_shell::CGALMeshShell> {
		using type = std::vector<cgal_utils::CGALMeshPointAndNormal>;
	};

	namespace distance_matrix {

		template<>
		std::vector<std::vector<double>>
		point_distance_all_to_all(const mesh_shell::CGALMeshShell &context,
								  const std::vector<cgal_utils::CGALMeshPointAndNormal> &points) {

			cgal_utils::WeightedMesh weightedMesh{context.tmesh, context.rotation_weight};

			return point_distance_all_to_all(
					weightedMesh,
					points
					);

		}

	}

}


#endif //MGODPL_CGALMESHSHELL_H
