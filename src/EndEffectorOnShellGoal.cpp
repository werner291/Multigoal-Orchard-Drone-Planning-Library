#include "EndEffectorOnShellGoal.h"
#include "ompl_custom.h"

#include <utility>

EndEffectorOnShellGoal::EndEffectorOnShellGoal(const ompl::base::SpaceInformationPtr &si,
											   OMPLSphereShellWrapper sphereShell,
											   Eigen::Vector3d focus)
		: GoalSampleableRegion(si), sphereShell(std::move(sphereShell)), focus(std::move(focus)) {
}

void EndEffectorOnShellGoal::sampleGoal(ompl::base::State *st) const {

	// Sample a point in R^3 in a gaussian distribution around the focus point.
	ompl::RNG rng;

	Eigen::Vector3d moved_focus(focus.x() + rng.gaussian(0.0, 0.5),
								focus.y() + rng.gaussian(0.0, 0.5),
								focus.z() + rng.gaussian(0.0, 0.5));

	// Then, generate a state on the shell, relying on the fact that the point will be projected onto the sphere.
	sphereShell.state_on_shell(moved_focus, st);
}

unsigned int EndEffectorOnShellGoal::maxSampleCount() const {
	// There isn't really a limit, but we return INT_MAX to indicate that as the next best thing.
	return INT_MAX;
}

double EndEffectorOnShellGoal::distanceGoal(const ompl::base::State *st) const {
	// Convert to a MoveIt state
	auto *state_space = si_->getStateSpace()->as<DroneStateSpace>();
	moveit::core::RobotState rs(state_space->getRobotModel());
	state_space->copyToRobotState(rs, st);

	// Compute end-effector position with forward kinematics
	Eigen::Vector3d ee_pos = rs.getGlobalLinkTransform("end_effector").translation();
	Eigen::Vector3d shell_projection = sphereShell.getShell()->project(ee_pos);

	// Return the Euclidean distance between the end-effector and the shell projection.
	return (shell_projection - ee_pos).norm();
}
