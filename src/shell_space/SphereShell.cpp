#include "SphereShell.h"

#include <utility>
#include <range/v3/all.hpp>

SphereShell::SphereShell(Eigen::Vector3d center, double radius, bool horizontalArm)
		: center(std::move(center)), radius(radius), horizontal_arm(horizontalArm) {
}

moveit::core::RobotState
SphereShell::state_on_shell(const moveit::core::RobotModelConstPtr &drone, const Eigen::Vector3d &a) const {
	// Check precondition that a is a point on the shell.
	assert((a - project(a)).squaredNorm() < 1e-6);

	Eigen::Vector3d required_facing = (center - a).normalized();

	if (horizontal_arm) {
		required_facing.z() = 0;
		required_facing.normalize();
	}

	return robotStateFromPointAndArmvec(drone, a, required_facing);

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
	double d = radius * acos(std::clamp(ra_ray.dot(rb_ray) / (radius * radius), -1.0, 1.0));

	return d;
}

Eigen::Vector3d SphereShell::project(const Eigen::Vector3d &a) const {
	// Simple central projection onto the shell.
	return center + (a - center).normalized() * radius;
}

Eigen::Vector3d SphereShell::gaussian_sample_near_point(const Eigen::Vector3d &near) const {

	ompl::RNG rng;

	return project(
			near + Eigen::Vector3d(rng.gaussian(0.0, 0.5),rng.gaussian(0.0, 0.5),rng.gaussian(0.0, 0.5))
			);

}

Eigen::Vector3d SphereShell::project(const moveit::core::RobotState &st) const {
	return project(st.getGlobalLinkTransform("end_effector").translation());
}

Eigen::Vector3d SphereShell::project(const Apple &st) const {
	return project(st.center);
}

moveit::core::RobotState robotStateFromFacing(const moveit::core::RobotModelConstPtr &drone,
											  const Eigen::Vector3d &desired_ee_pos,
											  const Eigen::Vector3d &required_facing) {
	moveit::core::RobotState st(drone);
	// A horrifying way to compute the reuired yaw angle.
	Eigen::Vector3d default_facing(0.0, 1.0, 0.0);
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
	Eigen::Vector3d offset = desired_ee_pos - st.getGlobalLinkTransform("end_effector").translation();

	st.setVariablePosition(0, offset.x());
	st.setVariablePosition(1, offset.y());
	st.setVariablePosition(2, offset.z());

	st.update(true);

	return st;
}


Eigen::Vector3d SphereSurfaceArmVector::arm_vector(const Eigen::Vector3d &p) const {
	return (center - p).normalized();
}

Eigen::Vector3d SphereSurfaceArmVector::nearest_point_on_shell(const Eigen::Vector3d &p) const {
	return (p - center).normalized() * radius + center;
}

Eigen::Vector3d SphereSurfaceArmVector::surface_point(const Eigen::Vector3d &p) const {
	return p;
}
