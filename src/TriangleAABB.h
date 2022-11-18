// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 18-11-22.
//

#ifndef NEW_PLANNERS_TRIANGLEAABB_H
#define NEW_PLANNERS_TRIANGLEAABB_H


#include <CGAL/Simple_cartesian.h>
#include <shape_msgs/msg/mesh.hpp>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <vtkPolyData.h>
#include "utilities/mesh_utils.h"
#include "exploration/SegmentedPointCloud.h"
#include "WorkspaceSpec.h"

/**
 * A wrapper around CGAL's AABB tree to make it easier to use.
 *
 * Takes a ROS mesh message and builds an AABB tree for it, then allows querying for the on_which_mesh triangle on the mesh to a given point.
 */
class TriangleAABB {

	using K = CGAL::Simple_cartesian<double>;
	using FT = K::FT;
	using Ray = K::Ray_3;
	using Line = K::Line_3;
	using Point = K::Point_3;
	using Triangle = K::Triangle_3;
	using Iterator = std::vector<Triangle>::iterator;
	using Primitive = CGAL::AABB_triangle_primitive<K, Iterator>;
	using AABB_triangle_traits = CGAL::AABB_traits<K, Primitive>;
	using Tree = CGAL::AABB_tree<AABB_triangle_traits>;

	std::vector<Triangle> triangles;
	Tree tree;

public:

	/**
	 * Build an AABB tree for the given mesh.
	 *
	 * @param mesh 		The mesh to build the tree for.
	 */
	explicit TriangleAABB(const shape_msgs::msg::Mesh &mesh);

	/**
	 * Find the on_which_mesh triangle on the mesh to the given point.
	 *
	 * Returns the ID of the on_which_mesh triangle (matching the mesh message) on the mesh to the given point and
	 * the distance to that point from the query point.
	 *
	 * @param point 		The point to find the on_which_mesh triangle to.
	 * @return 				The triangle Id and distance.
	 */
	std::pair<size_t, double> closest(Eigen::Vector3d &point) const;

};

/**
 * An acceleration structure for, given a vector of meshes, facilitating queries
 * of whether the given point is inside any of the meshes, and if so, which mesh.
 */
class PointOnMeshLookup {

	std::vector<size_t> triangle_to_fruit;

	TriangleAABB aabb;

public:
	explicit PointOnMeshLookup(const std::vector<shape_msgs::msg::Mesh> &meshes);

	std::optional<size_t> on_which_mesh(Eigen::Vector3d &point, double margin) const;
};

class PointSegmenter {

	PointOnMeshLookup lookup;

public:
	explicit PointSegmenter(const WorkspaceSpec& spec);

	SegmentedPointCloud::ByType segmentPointCloudData(vtkPolyData *pPolyData);
};


#endif //NEW_PLANNERS_TRIANGLEAABB_H
