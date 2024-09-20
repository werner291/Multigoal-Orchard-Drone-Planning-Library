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

#include "../math/Vec3.h"
#include "Mesh.h"

namespace mgodpl::cgal {

	// Make a CGAL convex hull.
	using K = CGAL::Exact_predicates_inexact_constructions_kernel;
	using Point_3 = K::Point_3;
	using Ray_3 = K::Ray_3;
	using Direction_3 = K::Direction_3;
	using Surface_mesh = CGAL::Surface_mesh<Point_3>;
	using Traits = CGAL::Surface_mesh_shortest_path_traits<K, Surface_mesh>;
	using Surface_mesh_shortest_path = CGAL::Surface_mesh_shortest_path<Traits>;
	using Primitive = CGAL::AABB_face_graph_triangle_primitive<Surface_mesh>;
	using AABBTraits = CGAL::AABB_traits<K, Primitive>;

	inline mgodpl::cgal::Point_3 to_cgal_point(const mgodpl::math::Vec3<double> &v) {
		return {v.x(), v.y(), v.z()};
	}

	inline mgodpl::cgal::Direction_3 to_cgal_direction(const mgodpl::math::Vec3<double> &v) {
		return {v.x(), v.y(), v.z()};
	}

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

	/**
	 * @brief Computes the convex hull around the leaves of a tree using CGAL.
	 *
	 * This function takes a mesh representing the leaves of a tree and computes the convex hull around them.
	 * The convex hull is represented as a CGAL Surface_mesh.
	 *
	 * @param leaves_mesh The mesh representing the leaves of the tree.
	 * @return The convex hull around the leaves as a CGAL Surface_mesh.
	 */
	mgodpl::cgal::Surface_mesh cgal_convex_hull_around_leaves(const Mesh &leaves_mesh);

	/**
	 * @struct CgalMeshData
	 * @brief A struct that encapsulates the CGAL mesh data of a tree.
	 *
	 * This struct contains a convex hull, a shortest path algorithm on the mesh, and an AABB tree.
	 * These are computed from a given tree model. The constructor takes a tree model
	 * and computes the convex hull, initializes the shortest path algorithm, and the AABB tree.
	 */
	struct CgalMeshData {
		mgodpl::cgal::Surface_mesh convex_hull; ///< The convex hull around the leaves of the tree.
		mgodpl::cgal::Surface_mesh_shortest_path mesh_path; ///< The shortest path algorithm on the surface mesh of the convex hull.
		CGAL::AABB_tree<AABBTraits> tree; ///< The AABB tree built from the surface mesh.

		/**
		 * @brief Constructor for the CgalMeshData struct.
		 *
		 * This constructor takes a tree model and computes the convex hull,
		 * initializes the shortest path algorithm, and the AABB tree.
		 *
		 * @param tree_model The tree model from which to compute the mesh data.
		 */
		explicit CgalMeshData(const Mesh &leaves_mesh);
	};

	Surface_mesh_shortest_path::Face_location locate_nearest(const math::Vec3d& pt, const CgalMeshData& data);

	struct SurfacePointAndNormal {
		math::Vec3d surface_point;
		math::Vec3d normal;
	};

	SurfacePointAndNormal from_face_location(const Surface_mesh_shortest_path::Face_location& fl, const CgalMeshData& data);

}

#endif //MGODPL_CGAL_CHULL_SHORTEST_PATHS_H
