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
#include "../AppleTreePlanningScene.h"

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Triangle_mesh = CGAL::Surface_mesh<Kernel::Point_3>;
using Traits = CGAL::Surface_mesh_shortest_path_traits<Kernel, Triangle_mesh>;
using Primitive = CGAL::AABB_face_graph_triangle_primitive<Triangle_mesh>;
using AABBTraits = CGAL::AABB_traits<Kernel, Primitive>;
using Surface_mesh_shortest_path = CGAL::Surface_mesh_shortest_path<Traits>;
using CGALMeshPoint = Surface_mesh_shortest_path::Face_location;

struct Point_path_visitor_wrapper;

struct CGALMeshShellPoint {
	CGALMeshPoint point;
	/// We use an explicit normal here, because the surface normal is poorly defined at the edges and vertices.
	Eigen::Vector3d normal;
};

/**
 * A ShellSpace based on the Surface_mesh_shortest_path module in CGAL.
 *
 * The "shell" is a triangle mesh that is (typically) the convex hull of the leaf vertices in the tree.
 *
 * It should be noted that no part of the code explicitly assumes mesh convexity,
 * so a concave mesh *might* work as-is, but this is untested.
 *
 * The surface normals of the mesh are a bit ill-defined at the edges and vertices; for nearest_point_on_shell,
 * the normal is the normalized offset vector from the nearest point to the projected point.
 *
 * Optionally, the user may specify a non-zero "padding" value, which will be used to inflate the mesh
 * along the normals. This is useful to add a safety margin to the shell.
 *
 * Paths across the shell are exact shortest-path geodesics between points.
 */
class CGALMeshShell : public WorkspaceShell<CGALMeshShellPoint> {

protected:

	/// The CGAL mesh (a halfedge datastructure) for topology-aware shortest-paths computation.
	Triangle_mesh tmesh;

	/// An AABB-tree for quick lookup of the on_which_mesh point on the mesh (including facet information)
	CGAL::AABB_tree<AABBTraits> tree {};

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

	explicit CGALMeshShell(Triangle_mesh mesh, double rotationWeight, double padding);

	/**
	 * For a given CGALMeshShellPoint (which really is a face index paired with barycentric coordinates),
	 * compute the normal vector of the face.
	 *
	 * Note: this is not the normal vector of the triangle, special cases like edges or vertices are not handled explicitly:
	 * we exclusively look at the face index in the CGALMeshShellPoint.
	 *
	 * @param near	The point to compute the normal for.
	 * @return		The normal vector.
	 */
	Eigen::Vector3d normalAt(const CGALMeshShellPoint &near) const;

	/**
	 * Returns an "arm vector", which really is just a normal vector.
	 *
	 * @param p 		The surface shell point to compute the arm vector for.
	 * @return 			The arm vector.
	 */
	Eigen::Vector3d arm_vector(const CGALMeshShellPoint &p) const override;

	/**
	 *
	 * Find a point on the surface of the mesh that is closest to the given point, including a normal vector.
	 *
	 * If this closest point is on an edge or vertex, the normal vector is the normalized offset vector from
	 * the nearest point to the projected point.
	 *
	 * @param p 		The point to project onto the mesh.
	 * @return 			A CGALMeshShellPoint closest to p.
	 */
	CGALMeshShellPoint nearest_point_on_shell(const Eigen::Vector3d &p) const override;

	/**
	 * Compute the carthesian coordinates of a CGALMeshShellPoint.
	 *
	 * @param p 		The point to compute the coordinates for.
	 * @return 			The carthesian coordinates.
	 */
	Eigen::Vector3d surface_point(const CGALMeshShellPoint &p) const override;

	/**
	 * Compute a path across the surface between two shell points.
	 *
	 * TODO: The behavior of this function with start/endpoints near edges and vertices is somewhat poorly defined,
	 * need to test whether it's really what we're looking for.
	 *
	 * @param from 		The start point.
	 * @param to 		The end point.
	 * @return 			A piecewise-linear path between the two points.
	 */
	std::shared_ptr<ShellPath<CGALMeshShellPoint>>
	path_from_to(const CGALMeshShellPoint &from, const CGALMeshShellPoint &to) const override;

	/**
	 * Compute the length of a path across the surface between two shell points,
	 * since a ShellPath does not contain actual information about the mesh, only
	 * references to points on the mesh surface.
	 *
	 * @param path 		The path to compute the length for.
	 * @return 			The length of the path.
	 */
	double path_length(const std::shared_ptr<ShellPath<CGALMeshShellPoint>> &path) const override;
};

std::shared_ptr<WorkspaceShell<CGALMeshShellPoint>>
convexHullAroundLeavesCGAL(const AppleTreePlanningScene &scene_info, double rotation_weight, double padding);

#endif //NEW_PLANNERS_CGALMESHSHELL_H
