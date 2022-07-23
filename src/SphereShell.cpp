#include "SphereShell.h"

#include <utility>
#include <range/v3/all.hpp>

SphereShell::SphereShell(Eigen::Vector3d center, double radius) : center(std::move(center)), radius(radius) {
}

moveit::core::RobotState
SphereShell::state_on_shell(const moveit::core::RobotModelConstPtr &drone, const Eigen::Vector3d &a) const {
	// Check precondition that a is a point on the shell.
	assert((a - project(a)).squaredNorm() < 1e-6);

	moveit::core::RobotState st(drone);

	// A horrifying way to compute the reuired yaw angle.
	Eigen::Vector3d default_facing(0.0, 1.0, 0.0);
	Eigen::Vector3d required_facing = (center - a).normalized();
	Eigen::Vector3d base_facing = (Eigen::Vector3d(required_facing.x(), required_facing.y(), 0.0)).normalized();
	double yaw = copysign(acos(default_facing.dot(base_facing)), default_facing.cross(base_facing).z());

	Eigen::Quaterniond qd(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

	// Set to all 0s, except for the orientation of the base.
	st.setVariablePositions({0.0, 0.0, 0.0,      // Position off the side of the tree
							 qd.x(), qd.y(), qd.z(), qd.w(),// Identity rotation
							 0.0, 0.0, 0.0, 0.0  // Arm straight out
							});

	st.update(true);

	// Apply a translation to the base to bring the end-effector to the desired position.
	Eigen::Vector3d offset = a - st.getGlobalLinkTransform("end_effector").translation();

	st.setVariablePosition(0, offset.x());
	st.setVariablePosition(1, offset.y());
	st.setVariablePosition(2, offset.z());

	st.update(true);

	return st;
}

std::vector<moveit::core::RobotState> SphereShell::path_on_shell(const moveit::core::RobotModelConstPtr &drone,
																 const Eigen::Vector3d &a,
																 const Eigen::Vector3d &b) const {

	// Check precondition that a and b are points on the shell.
	assert((a - project(a)).squaredNorm() < 1e-6);
	assert((b - project(b)).squaredNorm() < 1e-6);

	// Rays from the center
	Eigen::Vector3d ra_ray = a - center;
	Eigen::Vector3d rb_ray = b - center;

	// Compute a normal vector as a rotation axis
	Eigen::Vector3d normal = ra_ray.cross(rb_ray).normalized();

	// Get the length of the geodesic
	double angle = predict_path_length(a, b);

	// Compute how many samples to take. This is a bit... arbitrary.
	const auto num_states = (size_t) (10.0 * angle) + 1;

	return
		// Linear distribute to get interpolation points.
			ranges::views::linear_distribute(0.0, angle, (long) num_states) |
			// Map to states.
			ranges::views::transform([&](double theta) {

				// Rotate ra_ray around the normal.
				Eigen::Vector3d p = center + Eigen::AngleAxisd(angle, normal) * ra_ray.normalized() * radius;

				// Generate a shell state at that point.
				return state_on_shell(drone, p);

			}) | ranges::to_vector;

}

double SphereShell::predict_path_length(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const {

	// Check precondition that a and b are points on the shell.
	assert((a - project(a)).squaredNorm() < 1e-6);
	assert((b - project(b)).squaredNorm() < 1e-6);

	// Rays from the center
	Eigen::Vector3d ra_ray = a - center;
	Eigen::Vector3d rb_ray = b - center;

	// Compute angle between the two in radians (norm is known to equal radius)
	return radius * acos(ra_ray.dot(rb_ray) / (radius * radius));
}

Eigen::Vector3d SphereShell::project(const Eigen::Vector3d &a) const {
	// Simple central projection onto the shell.
	return center + (a - center).normalized() * radius;
}

OMPLSphereShellWrapper::OMPLSphereShellWrapper(std::shared_ptr<CollisionFreeShell> shell,
											   ompl::base::SpaceInformationPtr si)
		: shell(std::move(shell)), si(std::move(si)) {
}

void OMPLSphereShellWrapper::state_on_shell(const Eigen::Vector3d &a, ompl::base::State *st) const {

	auto state_space = std::dynamic_pointer_cast<ompl_interface::ModelBasedStateSpace>(si->getStateSpace());

	state_space->copyToOMPLState(st, shell->state_on_shell(state_space->getRobotModel(), a));
}

ompl::geometric::PathGeometric OMPLSphereShellWrapper::path_on_shell(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
	return omplPathFromMoveitTrajectory(shell->path_on_shell(si->getStateSpace()
																	 ->as<ompl_interface::ModelBasedStateSpace>()
																	 ->getRobotModel(), a, b), si);
}

std::shared_ptr<CollisionFreeShell> OMPLSphereShellWrapper::getShell() const {
	return shell;
}

void OMPLSphereShellWrapper::state_on_shell(const ompl::base::Goal *apple, ompl::base::State *st) const {
	state_on_shell(shell->project(apple->as<DroneEndEffectorNearTarget>()->getTarget()), st);
}

ompl::geometric::PathGeometric
OMPLSphereShellWrapper::path_on_shell(const ompl::base::Goal *a, const ompl::base::Goal *b) {
	return path_on_shell(shell->project(a->as<DroneEndEffectorNearTarget>()->getTarget()),
						 shell->project(b->as<DroneEndEffectorNearTarget>()->getTarget()));;
}

double OMPLSphereShellWrapper::predict_path_length(const ompl::base::Goal *a, const ompl::base::Goal *b) const {
	return shell->predict_path_length(shell->project(a->as<DroneEndEffectorNearTarget>()->getTarget()),
									  shell->project(b->as<DroneEndEffectorNearTarget>()->getTarget()));
}

double OMPLSphereShellWrapper::predict_path_length(const ompl::base::State *a,
												   const ompl::base::Goal *b) const {

	auto ss = si->getStateSpace()->as<DroneStateSpace>();
	moveit::core::RobotState st(ss->getRobotModel());
	ss->copyToRobotState(st, a);

	return shell->predict_path_length(
			shell->project(st.getGlobalLinkTransform("end_effector").translation()),

						   shell->project(b->as<DroneEndEffectorNearTarget>()->getTarget()));
}

