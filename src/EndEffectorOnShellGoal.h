#ifndef NEW_PLANNERS_ENDEFFECTORONSHELLGOAL_H
#define NEW_PLANNERS_ENDEFFECTORONSHELLGOAL_H

#include <ompl/base/goals/GoalSampleableRegion.h>
#include "shell_space/OmplShellSpace.h"

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
	OmplShellSpace<ShellPoint> sphereShell;

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
						   OmplShellSpace<ShellPoint> sphereShell,
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

	}

};


#endif //NEW_PLANNERS_ENDEFFECTORONSHELLGOAL_H
