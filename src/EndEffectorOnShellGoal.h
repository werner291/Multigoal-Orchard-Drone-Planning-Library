#ifndef NEW_PLANNERS_ENDEFFECTORONSHELLGOAL_H
#define NEW_PLANNERS_ENDEFFECTORONSHELLGOAL_H

#include <ompl/base/goals/GoalSampleableRegion.h>
#include "shell/MoveItAppleSphereShell.h"

/**
 * A GoalSampleableRegion that generates states states where the end-effector is on a given sphere shell,
 * in a roughly Gaussian distribution near a given reference point on the spherical shell.
 *
 * For example, this is used in approach-state optimization in combination with
 * the goal-resampling capabilities of PathSimplifier.
 */
class EndEffectorOnShellGoal : public ompl::base::GoalSampleableRegion {

	/// The sphere shell to sample on.
	MoveItAppleSphereShell sphereShell;

	// The focus point on the sphere shell to sample near.
	Eigen::Vector3d focus;

public:
	/**
	 * Constructor.
	 * @param si 			SpaceInformation reference
	 * @param sphereShell 	The sphere shell to sample on.
	 * @param focus 		The focus point on the sphere shell to sample near.
	 */
	EndEffectorOnShellGoal(const ompl::base::SpaceInformationPtr &si,
						   MoveItAppleSphereShell sphereShell,
						   Eigen::Vector3d focus);

	/**
	 * Sample a state on the sphere shell, near the target.
	 *
	 * @param st 	The state to fill in.
	 */
	void sampleGoal(ompl::base::State *st) const override;

	/**
	 * Maximum number of samples that can be drawn from this goal. Infinite, in our case, so we return INT_MAX.
	 */
	[[nodiscard]] unsigned int maxSampleCount() const override;

	/**
	 * Distance of a given state from the goal. In our case, that's
	 * the distance between the end-effector and the spherical shell.
	 *
	 * @param st 	The state to measure the distance from.
	 * @return 		The distance.
	 */
	double distanceGoal(const ompl::base::State *st) const override;

	const Eigen::Vector3d &getFocus() const;

	void setFocus(const Eigen::Vector3d &focus);

};


#endif //NEW_PLANNERS_ENDEFFECTORONSHELLGOAL_H
