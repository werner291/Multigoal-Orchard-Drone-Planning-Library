// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 28-6-23.
//

#ifndef MGODPL_CGAL_UTILS_H
#define MGODPL_CGAL_UTILS_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_shortest_path/Surface_mesh_shortest_path_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Surface_mesh_shortest_path.h>

#include "../pure/metric_space.h"

namespace mgodpl {

	namespace cgal_utils {

		using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
		using Triangle_mesh = CGAL::Surface_mesh<Kernel::Point_3>;
		using Traits = CGAL::Surface_mesh_shortest_path_traits<Kernel, Triangle_mesh>;
		using Primitive = CGAL::AABB_face_graph_triangle_primitive<Triangle_mesh>;
		using AABBTraits = CGAL::AABB_traits<Kernel, Primitive>;
		using Surface_mesh_shortest_path = CGAL::Surface_mesh_shortest_path<Traits>;
		using CGALMeshPoint = Surface_mesh_shortest_path::Face_location;
		using LocationTree = CGAL::AABB_tree<AABBTraits>;

		/**
		 * @struct CGALMeshPointAndNormal
		 * @brief This struct holds a CGALMeshPoint and a normal vector.
		 *
		 * The normal is stored explicitly because the surface normal can be poorly defined at the edges and vertices.
		 */
		struct CGALMeshPointAndNormal {
			CGALMeshPoint point;
			/// We use an explicit normal here, because the surface normal is poorly defined at the edges and vertices.
			Eigen::Vector3d normal;
		};

		/**
		 * @struct PathVisitor
		 * @brief Visitor for Surface_mesh_shortest_path::shortest_path_sequence_to_source_points to build a robot path.
	     */
		struct PathVisitor {

			const Triangle_mesh &mesh; ///< Reference to the triangle mesh
			std::vector<CGALMeshPoint> states; ///< The path being built.
			Surface_mesh_shortest_path path_algo; ///< Shortest path algorithm struct (for point lookups and such)

			/**
			 * @brief Constructor for PathVisitor struct.
			 * @param mesh The triangle mesh.
			 */
			explicit PathVisitor(const Triangle_mesh &mesh);

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
		 * @brief Converts a Kernel::Point_3 to an Eigen::Vector3d.
		 * @param p The point to convert.
		 * @return The converted point.
		 */
		Eigen::Vector3d toEigen(const Kernel::Point_3 &p);

		/**
		 * @brief Calculates the normal of a face on a given mesh.
		 * @param tmesh The mesh.
		 * @param face The face.
		 * @return The normal of the face.
		 */
		Eigen::Vector3d faceNormal(const Triangle_mesh &tmesh, const Surface_mesh_shortest_path::face_descriptor &face);

		/**
		 * @brief Locates the nearest point on a mesh to a given point.
		 * @param tmesh The mesh.
		 * @param pt The given point.
		 * @param tree The tree for nearest point search.
		 * @return The nearest point and its normal.
		 */
		CGALMeshPointAndNormal locate(const Triangle_mesh &tmesh, const Kernel::Point_3 &pt, const LocationTree &tree);

		/**
		 * @brief Converts a CGALMeshPoint to a point in Euclidean coordinates.
		 * @param tmesh The mesh.
		 * @param p The point to convert.
		 * @return The converted point.
		 */
		Kernel::Point_3 to_carthesian(const Triangle_mesh &tmesh, const CGALMeshPoint &p);

		/**
		 * @struct WeightedMesh
		 * @brief Represents a triangle mesh and a rotation weight.
		 *
		 * The weight is given to the angle of rotation between facets when computing the distance between two points on the mesh surface.
		 */
		struct WeightedMesh {
			const Triangle_mesh &mesh; ///< The mesh.
			double rotation_weight; ///< The rotation weight.
		};

		/**
		 * @brief Computes the length of a path on a weighted mesh.
		 * @param mesh The weighted mesh.
		 * @param path The path.
		 * @return The length of the path.
		 */
		double path_length(const WeightedMesh &mesh, const std::vector<CGALMeshPointAndNormal> &path);

	}

	template<>
	struct space_point_t<cgal_utils::WeightedMesh> {
		using type = cgal_utils::CGALMeshPointAndNormal;
	};

	namespace metric_space {
		template<>
		std::vector<std::vector<double>>
		point_distance_all_to_all(const cgal_utils::WeightedMesh &context,
								  const std::vector<cgal_utils::CGALMeshPointAndNormal> &points);
	}

}

#endif //MGODPL_CGAL_UTILS_H
