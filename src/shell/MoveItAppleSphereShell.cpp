
#include <range/v3/view/linear_distribute.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include "MoveItAppleSphereShell.h"
#include "../ompl_custom.h"

Eigen::Quaterniond yaw_facing_toward(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
	auto yaw = -atan2(b.y() - a.y(), b.x() - a.x());
	return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

ompl::base::ScopedState<> MoveItAppleSphereShell::state_on_shell(const Eigen::Vector3d &a) const {

	// Precondition: Point `a` must be on the sphere surface
	assert(abs((a-sphere.center).norm() - sphere.radius) < 1e-6);

	auto qd = yaw_facing_toward(a, sphere.center);

	moveit::core::RobotState st(state_space->getRobotModel());
	st.setVariablePositions({
									0.0, 0.0, 0.0,      // Position off the side of the tree
									qd.x(), qd.y(), qd.z(), qd.w(),
									0.0, 0.0, 0.0, 0.0  // Arm straight out
							});
	st.update(true);

	Eigen::Vector3d offset = a - st.getGlobalLinkTransform("end_effector").translation();

	st.setVariablePosition(0, offset.x());
	st.setVariablePosition(1, offset.y());
	st.setVariablePosition(2, offset.z());

	st.update(true);

	ompl::base::ScopedState<> state(state_space);

	state_space->copyToOMPLState(state.get(), st);

	return state;
}

Eigen::Vector3d MoveItAppleSphereShell::shell_point(const ompl::base::Goal *const &a) const {

	return project(a->as<DroneEndEffectorNearTarget>()->getTarget());

}

std::vector<ompl::base::ScopedState<>> MoveItAppleSphereShell::path_on_shell(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const {

	Eigen::Vector3d ra_ray = a - sphere.center;
	Eigen::Vector3d rb_ray = b - sphere.center;

	Eigen::Vector3d normal = ra_ray.cross(rb_ray).normalized();

	double angle = predict_path_length(a,b);

	const auto num_states = (size_t) (10.0 * angle) + 1;

	return
		ranges::views::linear_distribute(0.0, angle, (long)num_states) |
		ranges::views::transform([&](double t) {
			return state_on_shell(
					sphere.center + Eigen::AngleAxisd(angle * t, normal) * ra_ray.normalized() * sphere.radius);
		}) | ranges::to_vector;

}

double MoveItAppleSphereShell::predict_path_length(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const {

	Eigen::Vector3d ra_ray = a - sphere.center;
	Eigen::Vector3d rb_ray = b - sphere.center;

	return acos(ra_ray.dot(rb_ray) / (ra_ray.norm() * rb_ray.norm()));
}

Eigen::Vector3d MoveItAppleSphereShell::shell_point(const ompl::base::ScopedState<> &a) const {

	moveit::core::RobotState st(state_space->getRobotModel());
	state_space->copyToRobotState(st, a.get());
	st.update(true);

	return st.getGlobalLinkTransform("end_effector").translation();

}

Eigen::Vector3d MoveItAppleSphereShell::project(const Eigen::Vector3d &a) const {
	return sphere.center + sphere.radius * (a - sphere.center).normalized();
}
