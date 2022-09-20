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
 * A ShellBuilder that builds a CuttingPlaneConvexHullShell around the leaf vertices in a tree.
 */
class CGALConvexHullShellBuilder : public ShellPathPlanner<CGALMeshPoint>::ShellBuilder {

	/// When computing the path_on_shell, the states will be offset from the shell by this distance.
	double padding = 0.1;
	/// How heavy to weigh rotation in predict_path_length.
	double rotation_weight = 1.0;

public:

	/**
	 * Construct a CGALConvexHullShellBuilder.
	 *
	 * @param padding 			Offset of the states (end-effector) from the shell.
	 * @param rotationWeight 	How heavy to weigh rotation in predict_path_length.
	 */
	explicit CGALConvexHullShellBuilder(double padding, double rotationWeight);

	[[nodiscard]] std::shared_ptr<OMPLShellSpaceWrapper<CGALMeshPoint>>
	buildShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) const override;

	[[nodiscard]] Json::Value parameters() const override;
};

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
class CGALMeshShell : public CollisionFreeShell<CGALMeshPoint> {

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
	 * Construct a RobotState where the robot's end effector is on the shell at the given point,
	 * facing directly towards the shell. This state is guaranteed to be collision-free.
	 *
	 * @param drone 			The drone to construct the state for.
	 * @param a 				The point on the shell to place the end effector at.
	 * @return 					The constructed state.
	 */
	[[nodiscard]] moveit::core::RobotState state_on_shell(const moveit::core::RobotModelConstPtr &drone,
														  const CGALMeshPoint &a) const override;

	/**
	 * Plan a path of RobotStates from the given start to the given goal.
	 *
	 * Linear interpolation between the states in the returned vector are guaranteed to have the robot's end-effector
	 * on the convex hull at all times (+ padding), and the path is guaranteed to be collision-free.
	 *
	 * @param drone 	The robot model to use.
	 * @param start 	The start point.
	 * @param goal 		The goal point.
	 *
	 * @return 			A vector of RobotStates, which serve as anchor points for linear interpolation.
	 */
	[[nodiscard]] std::vector<moveit::core::RobotState> path_on_shell(const moveit::core::RobotModelConstPtr &drone,
														const CGALMeshPoint &a,
														const CGALMeshPoint &b) const override;

	/**
	 * Generate a ConvexHullPoint in a (roughly) gaussian distribution around another point.
	 *
	 * @param near 	The point to generate around.
	 * @return 		The generated point.
	 */
	[[nodiscard]] CGALMeshPoint gaussian_sample_near_point(const CGALMeshPoint &near) const override;

	/**
	 * Predict the length of a path from the given start to the given goal by generating and measuring the length
	 * of a segmented line across the surface without constructing all the robot states.
	 *
	 * @param a 	The start point.
	 * @param b 	The goal point.
	 * @return 		The predicted length of the path.
	 */
	[[nodiscard]] double predict_path_length(const CGALMeshPoint &a,
											 const CGALMeshPoint &b) const override;

	/**
	 * Find the CGALMeshPoint of the projection of the end-effector of the robot onto the convex hull.
	 *
	 * @param st 		The RobotState to use.
	 * @return 			The projected point.
	 */
	[[nodiscard]] CGALMeshPoint project(const moveit::core::RobotState &st) const override;

	/**
	 * Find the CGALMeshPoint of the projection of an apple onto the convex hull.
	 *
	 * @param st 		The RobotState to use.
	 * @return 			The projected point.
	 */
	[[nodiscard]] CGALMeshPoint project(const Apple &st) const override;

	/**
	 * Project a point onto the convex hull, returning a CGALMeshPoint (that includes the facet index).
	 *
	 * @param a 	The point to project.
	 * @return 		The projected point.
	 */
	[[nodiscard]] CGALMeshPoint project(const Eigen::Vector3d& pt) const;

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
};


#endif //NEW_PLANNERS_CGALMESHSHELL_H
