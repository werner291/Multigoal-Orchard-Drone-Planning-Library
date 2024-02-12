//
// Created by werner on 2/12/24.
//

#include "approach_path_planning.h"
#include "state_tools.h"

#include <CGAL/Polygon_mesh_processing/compute_normal.h>

using namespace mgodpl;
using namespace mgodpl::approach_planning;
using namespace cgal;

mgodpl::ApproachPath mgodpl::approach_planning::plan_initial_approach_path(const mgodpl::robot_model::RobotModel &robot,
																		   const mgodpl::RobotState &initial_state,
																		   const mgodpl::robot_model::RobotModel::LinkId flying_base,
																		   const CgalMeshData &mesh_data) {


	ApproachPath initial_approach_path;

	auto initial_fk = robot_model::forwardKinematics(robot, initial_state.joint_values,
													 flying_base, initial_state.base_tf);

	// The convex hull point closest to the initial state.
	const math::Vec3d &ee_pose = initial_fk.forLink(flying_base).translation;
	const auto &[initial_face, initial_bary] = mesh_data.mesh_path.locate(Point_3(ee_pose.x(), ee_pose.y(), ee_pose.z()),
																		  mesh_data.tree);
	auto face_normal = CGAL::Polygon_mesh_processing::compute_face_normal(initial_face, mesh_data.convex_hull);
	auto pt = mesh_data.mesh_path.point(initial_face, initial_bary);

	// Put the robot state outside the tree in that point:
	auto robot_state = fromEndEffectorAndVector(robot,
												{pt.x(), pt.y(), pt.z()},
												{face_normal.x(), face_normal.y(), face_normal.z()});

	initial_approach_path.path.states = {initial_state, robot_state};
	initial_approach_path.shell_point = {initial_face, initial_bary};

	return initial_approach_path;
}

mgodpl::ApproachPath mgodpl::approach_planning::straight_in_motion(const mgodpl::robot_model::RobotModel &robot,
																   const mgodpl::cgal::CgalMeshData &mesh_data,
																   const mgodpl::math::Vec3d &tgt) {

	// Find the nearest point on the hull.
	const auto &[face, barycentric] = mesh_data.mesh_path.locate(Point_3(tgt.x(), tgt.y(), tgt.z()), mesh_data.tree);

	// Find the normal vector:
	auto face_normal = CGAL::Polygon_mesh_processing::compute_face_normal(face, mesh_data.convex_hull);

	// Find the Cartesian coordinates of the point on the hull.
	const auto &pt = mesh_data.mesh_path.point(face, barycentric);

	// Put the robot state outside the tree in that point:
	auto robot_state = fromEndEffectorAndVector(robot,
												{pt.x(), pt.y(), pt.z()},
												{face_normal.x(), face_normal.y(), face_normal.z()});

	// Create a copy that has the end-effector at the target.
	auto robot_state_at_target = fromEndEffectorAndVector(robot,
														  tgt,
														  {face_normal.x(), face_normal.y(), face_normal.z()});

	// Create a path from the current state to the target state.
	ApproachPath path{
			.path = {.states={robot_state, robot_state_at_target}}, // TODO: actually compute the path.
			.shell_point = {face, barycentric}
	};

	return path;
}
