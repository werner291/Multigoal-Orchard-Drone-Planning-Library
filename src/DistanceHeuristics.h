#ifndef NEW_PLANNERS_DISTANCEHEURISTICS_H
#define NEW_PLANNERS_DISTANCEHEURISTICS_H

#include <ompl/base/State.h>
#include <ompl/base/Goal.h>
#include "ompl_custom.h"
#include "greatcircle.h"

/**
 * Base class for some notion of distance heuristics between either a state and a goal, or two goals.
 */
class OmplDistanceHeuristics {

public:
	/**
	 * Compute a heuristic distance between a given state and a goal.
	 */
	virtual double state_to_goal(const ompl::base::State *, const ompl::base::Goal *) const = 0;

	/**
	 * Compute a heuristic distance between two goals.
	 */
	virtual double goal_to_goal(const ompl::base::Goal *, const ompl::base::Goal *) const = 0;

	/**
	 * Get the name of this heuristic.
	 */
	[[nodiscard]] virtual std::string name() const = 0;
};

/**
 * Heuristic that uses the Euclidean distance. Assumes that the robot is a MoveIt robot model,
 * and has a link called "end-effector", and that goals are DroneEndEffectorNearTarget.
 */
class EuclideanOmplDistanceHeuristics : public OmplDistanceHeuristics {

	/// Keep a reference to the DroneStateSpace so that we can easily convert.
	std::shared_ptr<DroneStateSpace> state_space_;

public:
	/**
	 * Constructor.
	 * @param stateSpace Reference to the state space to convert between OMPL and MoveIt.
	 */
	EuclideanOmplDistanceHeuristics(std::shared_ptr<DroneStateSpace> stateSpace);

	/**
	 * Compute a heuristic distance between a given state and a goal. In this case, that is the Euclidean distance
	 * between the end-effector of the robot and the goal.
	 *
	 * @param a The state of the robot.
	 * @param b The goal, assumed to be a DroneEndEffectorNearTarget.
	 *
	 * @return The computed distance.
	 */
	double state_to_goal(const ompl::base::State *a, const ompl::base::Goal *b) const override;

	/**
	 * Compute a heuristic distance between two goals. In this case, that is the Euclidean distance
	 * between the two goals, assumed to be DroneEndEffectorNearTarget.
	 *
	 * @param a 	The first goal.
	 * @param b 	The second goal.
	 * @return 		The computed distance.
	 */
	double goal_to_goal(const ompl::base::Goal *a, const ompl::base::Goal *b) const override;

	/**
	 * @return The name of the heuristic ( "Euclidean" )
	 */
	[[nodiscard]] std::string name() const override;

};

/**
 * Heuristic that uses the Great Circle distance. Assumes that the robot is a MoveIt robot model,
 * and has a link called "end-effector", and that goals are DroneEndEffectorNearTarget.
 */
class GreatCircleOmplDistanceHeuristics : public OmplDistanceHeuristics {

	/// Underlying GreatCircleMetric to compute the great-circle distance between two points in R^3.
	GreatCircleMetric gcm;

	/// Keep a reference to the DroneStateSpace so that we can easily convert.
	std::shared_ptr<DroneStateSpace> state_space_;

public:

	/**
	 * Constructor.
	 * @param gcm 			The GreatCircleMetric to use.
	 * @param stateSpace 	Reference to the state space to convert between OMPL and MoveIt.
	 */
	GreatCircleOmplDistanceHeuristics(GreatCircleMetric gcm, std::shared_ptr<DroneStateSpace> stateSpace);

	/**
	 * Compute a heuristic distance between a given state and a goal. In this case, that is the Great Circle distance
	 * between the end-effector of the robot and the goal.
	 *
	 * @param a 	The state of the robot.
	 * @param b  	The goal, assumed to be a DroneEndEffectorNearTarget.
	 * @return 		The computed distance.
	 */
	double state_to_goal(const ompl::base::State *a, const ompl::base::Goal *b) const override;

	/**
	 * Compute the heuristic distance between two goals. In this case, that is the Great Circle distance
	 * between the two goals, assumed to be DroneEndEffectorNearTarget.
	 *
	 * @param a 	The first goal.
	 * @param b 	The second goal.
	 *
	 * @return 		The computed distance.
	 */
	double goal_to_goal(const ompl::base::Goal *a, const ompl::base::Goal *b) const override;

	/**
	 * @return The name of the heuristic ( "GreatCircle" )
	 */
	[[nodiscard]] std::string name() const override;

};

#endif //NEW_PLANNERS_DISTANCEHEURISTICS_H
