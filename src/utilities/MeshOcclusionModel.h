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

public:

	/**
	 * Constructor. Creates a mesh occlusion model from a mesh.
	 *
	 * @param mesh The mesh to create the occlusion model from
	 */
	explicit MeshOcclusionModel(const shape_msgs::msg::Mesh &mesh);

	/**
	 * Checks whether a point is occluded by the mesh, from a given viewpoint.
	 *
	 * @param point The point to check
	 *
	 * @param viewpoint The viewpoint from which to check occlusion
	 *
	 * @return True if the point is occluded, false otherwise
	 */
	[[nodiscard]] bool checkOcclusion(const Eigen::Vector3d &point,
									  const Eigen::Vector3d &viewpoint) const;

};


#endif //NEW_PLANNERS_MESHOCCLUSIONMODEL_H
