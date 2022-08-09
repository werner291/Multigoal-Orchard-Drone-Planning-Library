#ifndef NEW_PLANNERS_ENDEFFECTORONSHELLGOAL_H
#define NEW_PLANNERS_ENDEFFECTORONSHELLGOAL_H

#include <ompl/base/goals/GoalSampleableRegion.h>
#include "collision_free_shell/SphereShell.h"

/**
 * A GoalSampleableRegion that generates states states where the end-effector is on a given sphere shell,
 * in a roughly Gaussian distribution near a given reference point on the spherical shell.
 *
 * For example, this is used in approach-state optimization in combination with
 * the goal-resampling capabilities of PathSimplifier.
 */
template<typename ShellPoint>
class EndEffectorOnShellGoal : public ompl::base::GoalSampleableRegion {

	/// The sphere shell to sample on.
	OMPLSphereShellWrapper<ShellPoint> sphereShell;

	// The focus point on the sphere shell to sample near.
	ShellPoint focus;

public:
	/**
	 * Constructor.
	 * @param si 			SpaceInformation reference
	 * @param sphereShell 	The sphere shell to sample on.
	 * @param focus 		The focus point on the sphere shell to sample near.
	 */
	EndEffectorOnShellGoal(const ompl::base::SpaceInformationPtr &si,
						   OMPLSphereShellWrapper<ShellPoint> sphereShell,
						   ShellPoint focus) : GoalSampleableRegion(si), sphereShell(std::move(sphereShell)), focus(std::move(focus)) {
	}

	/**
	 * Sample a state on the sphere shell, near the target.
	 *
	 * @param st 	The state to fill in.
	 */
	void sampleGoal(ompl::base::State *st) const override {

		// Then, generate a state on the shell, relying on the fact that the point will be projected onto the sphere.
		sphereShell.state_on_shell(sphereShell.gaussian_sample_near_point(focus), st);
	}

	/**
	 * Maximum number of samples that can be drawn from this goal. Infinite, in our case, so we return INT_MAX.
	 */
	[[nodiscard]] unsigned int maxSampleCount() const override {
		// There isn't really a limit, but we return INT_MAX to indicate that as the next best thing.
		return INT_MAX;
	}

	/**
	 * Distance of a given state from the goal. In our case, that's
	 * the distance between the end-effector and the spherical shell.
	 *
	 * @param st 	The state to measure the distance from.
	 * @return 		The distance.
	 */
	double distanceGoal(const ompl::base::State *st) const override {

		throw std::runtime_error("Not implemented");

//		// Convert to a MoveIt state
//		auto *state_space = si_->getStateSpace()->as<DroneStateSpace>();
//		moveit::core::RobotState rs(state_space->getRobotModel());
//		state_space->copyToRobotState(rs, st);
//
//		// Compute end-effector position with forward kinematics
//		Eigen::Vector3d ee_pos = rs.getGlobalLinkTransform("end_effector").translation();
//		Eigen::Vector3d shell_projection = sphereShell.getShell()->project(rs);
//
//		// Return the Euclidean distance between the end-effector and the shell projection.
//		return (shell_projection - ee_pos).norm();
	}

};


#endif //NEW_PLANNERS_ENDEFFECTORONSHELLGOAL_H
