// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 7-11-23.
//

#ifndef MGODPL_CGAL_CHULL_SHORTEST_PATHS_H
#define MGODPL_CGAL_CHULL_SHORTEST_PATHS_H

// TODO: Can I use forward declarations here instead of including the whole header?
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_shortest_path/Surface_mesh_shortest_path_traits.h>
#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits.h>

namespace mgodpl::cgal {


	// Make a CGAL convex hull.
	using K = CGAL::Exact_predicates_inexact_constructions_kernel;
	using Point_3 = K::Point_3;
	using Surface_mesh = CGAL::Surface_mesh<Point_3>;
	using Traits = CGAL::Surface_mesh_shortest_path_traits<K, Surface_mesh>;
	using Surface_mesh_shortest_path = CGAL::Surface_mesh_shortest_path<Traits>;
	using Primitive = CGAL::AABB_face_graph_triangle_primitive<Surface_mesh>;
	using AABBTraits = CGAL::AABB_traits<K, Primitive>;

	/**
	 * @struct PathVisitor
	 * @brief Visitor for Surface_mesh_shortest_path::shortest_path_sequence_to_source_points to build a robot path.
	 */
	struct PathVisitor {

		const Surface_mesh &mesh; ///< Reference to the triangle mesh
		Surface_mesh_shortest_path &path_algo; ///< Shortest path algorithm struct (for point lookups and such)
		std::vector<Surface_mesh_shortest_path::Face_location> &states; ///< The path being built.

		/**
		 * @brief Called when the path leaves a face through a half-edge.
		 * @param edge The half-edge for leaving the face (NOT the opposite half-edge where we enter the face!)
		 * @param t Interpolation value between the two vertices of the half-edge where the intersection occurs.
		 */
		void operator()(Surface_mesh_shortest_path::halfedge_descriptor edge, Surface_mesh_shortest_path::FT t);

		/**
		 * @brief Called when the path *exactly* crosses an edge.
		 * @param vertex The vertex of the edge where the path crosses.
		 */
		void operator()(Surface_mesh_shortest_path::vertex_descriptor vertex);

		/**
		 * @brief Called when the path includes a point on the interior of a face.
		 * @param f The face where the path has a point.
		 * @param location The location of the point on the face (in barycentric coordinates)
		 */
		void operator()(Surface_mesh_shortest_path::face_descriptor f,
						Surface_mesh_shortest_path::Barycentric_coordinates location);
	};

}

#endif //MGODPL_CGAL_CHULL_SHORTEST_PATHS_H
