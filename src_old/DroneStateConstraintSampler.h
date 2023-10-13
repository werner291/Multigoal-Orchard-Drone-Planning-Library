#ifndef NEW_PLANNERS_DRONESTATECONSTRAINTSAMPLER_H
#define NEW_PLANNERS_DRONESTATECONSTRAINTSAMPLER_H

/**
 * Computes the end-effector position of a drone given the current state of the drone, then applies a translation
 * to the base such that the end-effector is within `tolerance` distance of the target point. Within this bound,
 * the distance is randomized.
 *
 * @param state 	The state of the drone, to be moved to the target (will be mutated)
 * @param tolerance Maximum distance between the end-effector and the target point.
 * @param target 	The target point.
 */
void moveEndEffectorToGoal(moveit::core::RobotState &state, double tolerance, const Eigen::Vector3d &target);

/**
 *
 * Assuming a drone model, will uniformly sample a robot state, with the following guarantees:
 *
 * 1. The translational component of the state is within a box of span:
 * 		[ - translation_bound, translation_bound ]
 * 		[ - translation_bound, translation_bound ]
 * 		[ 0.0, translation_bound ] (It will not be underground)
 *
 * 2. The floating base is upright (pitch and roll are zero)
 *
 * @param state 			The state (to be written to)
 * @param translation_bound The bound on the translational component of the state (see above)
 */
void randomizeUprightWithBase(moveit::core::RobotState &state, double translation_bound);

#endif //NEW_PLANNERS_DRONESTATECONSTRAINTSAMPLER_H
