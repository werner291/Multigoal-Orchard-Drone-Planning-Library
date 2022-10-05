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
#include <json/value.h>

#include "SphereShell.h"
#include "CuttingPlaneConvexHullShell.h"
#include "../planning_scene_diff_message.h"

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Triangle_mesh = CGAL::Surface_mesh<Kernel::Point_3>;
using Traits = CGAL::Surface_mesh_shortest_path_traits<Kernel, Triangle_mesh>;
using Primitive = CGAL::AABB_face_graph_triangle_primitive<Triangle_mesh>;
using AABBTraits = CGAL::AABB_traits<Kernel, Primitive>;
using Surface_mesh_shortest_path = CGAL::Surface_mesh_shortest_path<Traits>;
using CGALMeshPoint = Surface_mesh_shortest_path::Face_location;

struct Point_path_visitor_wrapper;

/**
 * A ShellSpace based on the Surface_mesh_shortest_path module in CGAL.
 *
 * The "shell" is a triangle mesh that is (typically) the convex hull of the leaf vertices in the tree.
 *
 * It should be noted that no part of the code explicitly assumes mesh convexity,
 * so a concave mesh *might* work as-is, but this is untested.
 *
 * Paths across the shell are exact shortest-path geodesics between points.
 */
class CGALMeshShell : public WorkspaceShell<CGALMeshPoint> {

protected:

	/// The CGAL mesh (a halfedge datastructure) for topology-aware shortest-paths computation.
	Triangle_mesh tmesh;

	/// An AABB-tree for quick lookup of the closest point on the mesh (including facet information)
	CGAL::AABB_tree<AABBTraits> tree;

	/// When computing the path_on_shell, the states will be offset from the shell by this distance.
	double padding = 0.1;

	/// How heavy to weigh rotation in predict_path_length.
	double rotation_weight = 1.0;

public:

	/**
	 * Construct a CGALMeshShell.
	 *
	 * @param mesh 				A mesh to compute paths on. (Triangles must use consistent winding order.)
	 * @param rotationWeight 	How heavy to weigh rotation in predict_path_length.
	 * @param padding 			Offset of the states (at the end-effector) from the shell.
	 */
	explicit CGALMeshShell(const shape_msgs::msg::Mesh &mesh, double rotationWeight, double padding);

	/**
	 * For a given CGALMeshPoint (which really is a face index paired with barycentric coordinates),
	 * compute the carthesian coordinates.
	 *
	 * @param pt
	 * @return
	 */
	Eigen::Vector3d toCarthesian(const CGALMeshPoint &pt) const;

	/**
	 * For a given CGALMeshPoint (which really is a face index paired with barycentric coordinates),
	 * compute the normal vector of the face.
	 *
	 * Note: this is not the normal vector of the triangle, special cases like edges or vertices are not handled explicitly:
	 * we exclusively look at the face index in the CGALMeshPoint.
	 *
	 * @param near	The point to compute the normal for.
	 * @return		The normal vector.
	 */
	Eigen::Vector3d normalAt(const CGALMeshPoint &near) const;

	Eigen::Vector3d arm_vector(const CGALMeshPoint &p) const override;

	CGALMeshPoint nearest_point_on_shell(const Eigen::Vector3d &p) const override;

	Eigen::Vector3d surface_point(const CGALMeshPoint &p) const override;

	std::shared_ptr<ShellPath<CGALMeshPoint>>
	path_from_to(const CGALMeshPoint &from, const CGALMeshPoint &to) const override;

	double path_length(const std::shared_ptr<ShellPath<CGALMeshPoint>> &path) const override;
};

std::shared_ptr<WorkspaceShell<CGALMeshPoint>> convexHullAroundLeavesCGAL(
		const AppleTreePlanningScene &scene_info,
		const ompl::base::SpaceInformationPtr &si,
		double rotation_weight,
		double padding);

#endif //NEW_PLANNERS_CGALMESHSHELL_H