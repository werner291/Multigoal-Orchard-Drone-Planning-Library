//
// Created by werner on 12-8-22.
//

#ifndef NEW_PLANNERS_CGALMESHSHELL_H
#define NEW_PLANNERS_CGALMESHSHELL_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/Random.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>

#include "SphereShell.h"

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Triangle_mesh = CGAL::Surface_mesh<Kernel::Point_3>;
using Traits = CGAL::Surface_mesh_shortest_path_traits<Kernel, Triangle_mesh>;
using Primitive = CGAL::AABB_face_graph_triangle_primitive<Triangle_mesh>;
using AABBTraits = CGAL::AABB_traits<Kernel, Primitive>;
using Surface_mesh_shortest_path = CGAL::Surface_mesh_shortest_path<Traits>;
using CGALMeshPoint = Surface_mesh_shortest_path::Face_location;

struct Point_path_visitor_wrapper;

class CGALMeshShell : public CollisionFreeShell<CGALMeshPoint> {

	friend Point_path_visitor_wrapper;

protected:
	Triangle_mesh tmesh;
	CGAL::AABB_tree<AABBTraits> tree;
	double padding = 0.1;

public:

	explicit CGALMeshShell(const shape_msgs::msg::Mesh &mesh);

	[[nodiscard]] moveit::core::RobotState state_on_shell(const moveit::core::RobotModelConstPtr &drone,
														  const CGALMeshPoint &a) const override;

	[[nodiscard]] std::vector<moveit::core::RobotState> path_on_shell(const moveit::core::RobotModelConstPtr &drone,
														const CGALMeshPoint &a,
														const CGALMeshPoint &b) const override;

	[[nodiscard]] CGALMeshPoint gaussian_sample_near_point(const CGALMeshPoint &near) const override;

	[[nodiscard]] double predict_path_length(const CGALMeshPoint &a,
											 const CGALMeshPoint &b) const override;

	[[nodiscard]] CGALMeshPoint project(const moveit::core::RobotState &st) const override;

	[[nodiscard]] CGALMeshPoint project(const Apple &st) const override;

	[[nodiscard]] CGALMeshPoint project(const Eigen::Vector3d& pt) const;

	Eigen::Vector3d toCarthesian(const CGALMeshPoint &near) const;

	Eigen::Vector3d normalAt(const CGALMeshPoint &near) const;
};


#endif //NEW_PLANNERS_CGALMESHSHELL_H
