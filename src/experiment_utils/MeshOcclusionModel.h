//
// Created by werner on 27-2-23.
//

#ifndef NEW_PLANNERS_MESHOCCLUSIONMODEL_H
#define NEW_PLANNERS_MESHOCCLUSIONMODEL_H

#include <shape_msgs/msg/mesh.hpp>
#include <Eigen/Core>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

#include "../math/Vec3.h"

namespace mgodpl {
	class MeshOcclusionModel {

		using K = CGAL::Simple_cartesian<double>;
		using FT = K::FT;
		using Ray = K::Ray_3;
		using Segment = K::Segment_3;
		using Point = K::Point_3;
		using Triangle = K::Triangle_3;
		using Iterator = std::vector<Triangle>::iterator;
		using Primitive = CGAL::AABB_triangle_primitive<K, Iterator>;
		using AABB_triangle_traits = CGAL::AABB_traits<K, Primitive>;
		using Tree = CGAL::AABB_tree<AABB_triangle_traits>;

		Tree tree;
		std::vector<Triangle> triangles;

		double margin = 0.05;

	public:

		/**
		 * Constructor. Creates a mesh occlusion model from a mesh.
		 *
		 * @param mesh The mesh to create the occlusion model from
		 */
		MeshOcclusionModel(const shape_msgs::msg::Mesh &mesh, double margin);

		/**
		 * Checks whether a point is occluded by the mesh, from a given viewpoint.
		 *
		 * @param point The point to check
		 *
		 * @param viewpoint The viewpoint from which to check occlusion
		 *
		 * @return True if the point is occluded, false otherwise
		 */
		[[nodiscard]] bool checkOcclusion(const math::Vec3d &point, const math::Vec3d &viewpoint) const;

		/**
		 * Computes the exterior visibility score of an apple based on occlusion tests
		 * @param apple - the position of the apple in 3D space
		 * @return a score between 0 and 1 representing the fraction of random lines from the apple that are not occluded
		 */
		double exteriorVisibilityScore(const math::Vec3d &apple, const int n_samples);

	};
}

#endif //NEW_PLANNERS_MESHOCCLUSIONMODEL_H
