#include "MoveItShellSpace.h"
#include "../utilities/moveit.h"
#include "CylinderShell.h"
#include "CuttingPlaneConvexHullShell.h"
#include "CGALMeshShell.h"

struct Polar {
	double r;
	double azimuth;
	double altitude;
};

Polar pointToPolar(const Eigen::Vector3d &point) {
	Polar polar{};
	polar.r = point.norm();
	polar.azimuth = std::atan2(point.y(), point.x());
	polar.altitude = std::asin(point.z() / polar.r);
	return polar;
}

moveit::core::RobotState robotStateFromPointAndArmvec(const moveit::core::RobotModelConstPtr &drone,
													  const Eigen::Vector3d &desired_ee_pos,
													  const Eigen::Vector3d &armvec) {

	// Ensure armvec is normalized
	assert(std::abs(armvec.norm() - 1.0) < 1e-6);

	Polar polar = pointToPolar(armvec);

	Eigen::Quaterniond qd(Eigen::AngleAxisd(2.0 * M_PI - (M_PI / 2.0 - polar.azimuth), Eigen::Vector3d::UnitZ()));

	// Pick a base joint angle such that the arm is pointing in the right direction.

	moveit::core::RobotState st(drone);

	st.setVariablePositions({0.0, 0.0, 0.0,
							 qd.x(), qd.y(), qd.z(), qd.w(),
							 polar.altitude,
							 0.0,
							 0.0,
							 0.0
							});

	st.update(true);

	// Apply a translation to the base to bring the end-effector to the desired position.

	setBaseTranslation(st, desired_ee_pos - st.getGlobalLinkTransform("end_effector").translation());

	st.update(true);

	// Sanity check: is the arm pointing in the right direction?
	{
		Eigen::Vector3d actual_armvec = st.getGlobalLinkTransform("arm2").translation() - st.getGlobalLinkTransform("arm").translation();
		actual_armvec.normalize();

		assert((actual_armvec - armvec).norm() < 1e-6);

		Eigen::Vector3d actual_ee_pos = st.getGlobalLinkTransform("end_effector").translation();
		assert((actual_ee_pos - desired_ee_pos).norm() < 1e-6);
	}

	return st;

}


template<typename ShellPoint>
ShellPoint MoveItShellSpace<ShellPoint>::pointNearGoal(const Apple &apple) const {
	return shell->nearest_point_on_shell(apple.center);
}

template<typename ShellPoint>
ShellPoint MoveItShellSpace<ShellPoint>::pointNearState(const moveit::core::RobotState &state) const {
	return shell->nearest_point_on_shell(state.getGlobalLinkTransform("end_effector").translation());
}

template<typename ShellPoint>
double MoveItShellSpace<ShellPoint>::predict_path_length(const ShellPoint &start, const ShellPoint &end) const {
	return shell->path_length(shell->path_from_to(start, end));
}

template<typename ShellPoint>
std::vector<std::vector<double>>
MoveItShellSpace<ShellPoint>::distance_matrix(const std::vector<ShellPoint> &points) const {
	return shell->distance_matrix(points);
}

template<typename ShellPoint>
RobotPath MoveItShellSpace<ShellPoint>::shellPath(const ShellPoint &start, const ShellPoint &end) const {

	// Get the path from the workspace shell.
	auto path = shell->path_from_to(start, end);

	// Convert the path to a robot path.
	RobotPath robot_path;

	if (auto piecewise = std::dynamic_pointer_cast<const PiecewiseLinearPath<ShellPoint>>(path)) {

		assert(!piecewise->points.empty());

		// Initial points
		Eigen::Vector3d prev_armvec = shell->arm_vector(piecewise->points.front());
		Eigen::Vector3d prev_surfacePoint = shell->surface_point(piecewise->points.front());
		robot_path.waypoints.push_back(robotStateFromPointAndArmvec(robot_model, prev_surfacePoint, prev_armvec));

		// Subsequent points:
		for (size_t i = 1; i < piecewise->points.size(); i++) {
			Eigen::Vector3d armvec = shell->arm_vector(piecewise->points[i]);
			Eigen::Vector3d surfacePoint = shell->surface_point(piecewise->points[i]);

			double angle = std::acos(std::clamp(armvec.dot(prev_armvec), -1.0, 1.0));

			auto num_steps = 1;// + (size_t) std::floor(3.0 * angle);

			for (size_t j = 1; j <= num_steps; j++) {
				double t = (double) j / (double) num_steps;
				Eigen::Vector3d armvec_t = (prev_armvec + t * (armvec - prev_armvec)).normalized();
				Eigen::Vector3d surfacePoint_t = prev_surfacePoint + t * (surfacePoint - prev_surfacePoint);
				robot_path.waypoints.push_back(robotStateFromPointAndArmvec(robot_model, surfacePoint_t, armvec_t));
			}

			prev_armvec = armvec;
			prev_surfacePoint = surfacePoint;
		}

		return robot_path;
	}
	if (auto curve = std::dynamic_pointer_cast<const CurvePath<ShellPoint>>(path)) {
		// Just take 100 samples.
		for (size_t i = 0; i <= 100; i++) {
			double t = (double) i / 100.0;
			ShellPoint point = curve->at(t);
			robot_path.waypoints.push_back(stateFromPoint(point));
		}
	} else {
		throw std::runtime_error("Unknown path type.");
	}

	return robot_path;
}

template<typename ShellPoint>
moveit::core::RobotState MoveItShellSpace<ShellPoint>::stateFromPoint(const ShellPoint &point) const {

	Eigen::Vector3d armvec = shell->arm_vector(point);
	Eigen::Vector3d surface_point = shell->surface_point(point);

	return robotStateFromPointAndArmvec(robot_model, surface_point, armvec);

}

template<typename ShellPoint>
MoveItShellSpace<ShellPoint>::MoveItShellSpace(moveit::core::RobotModelConstPtr robotModel,
											   const std::shared_ptr<const WorkspaceShell<ShellPoint>> &shell)
		: robot_model(std::move(robotModel)), shell(shell) {
}

// Explicit template instantiation
template
class MoveItShellSpace<Eigen::Vector3d>; // For the sphere shell
template
class MoveItShellSpace<CylinderShellPoint>; // For the cylinder shell
template
class MoveItShellSpace<ConvexHullPoint>; // For the cutting plane convex hull shell
template
class MoveItShellSpace<CGALMeshShellPoint>; // For the CGAL mesh shell