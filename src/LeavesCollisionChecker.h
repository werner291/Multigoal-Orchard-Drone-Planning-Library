//
// Created by werner on 09-09-21.
//

#ifndef NEW_PLANNERS_LEAVESCOLLISIONCHECKER_H
#define NEW_PLANNERS_LEAVESCOLLISIONCHECKER_H

#include <fcl/fcl.h>
#include <moveit/robot_state/robot_state.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <Eigen/Core>

/**
 * A class that checks if a robot state is in collision with the leaves of a tree,
 * represented as a "triangle soup" such that no assumptions need to be made about
 * convexity. (Leaves rarely are convex, after all.)
 *
 * TODO: only checks for collisions with the base link right now.
 */
class LeavesCollisionChecker {

	// A BVGModel that allows fast collision checking of the leaves.
	fcl::BVHModel<fcl::OBBRSSd> leaves;

public:
	/**
	 * Constructor.
	 * @param leaf_vertices The vertices of the leaves, assumed to be triples of triangle points.
	 */
	LeavesCollisionChecker(const std::vector<Eigen::Vector3d> &leaf_vertices);

	/**
	 * Checks if a robot state is in collision with the leaves.
	 * @param state 	The state to check.
	 * @return 			Set of leaf indices in collision (to allow for de-duplication)
	 */
	std::set<size_t> checkLeafCollisions(moveit::core::RobotState &state) const;

};

/**
 * An OMPL optimization objective to minimize collisions with leaves.
 */
class LeavesCollisionCountObjective : public ompl::base::StateCostIntegralObjective {

	/// The MoveIt robot model.
	const std::shared_ptr<const moveit::core::RobotModel> robot;

	/// The LeavesCollisionChecker to use contaning our collision model.
	const std::shared_ptr<const LeavesCollisionChecker> leaves;

public:
	/**
	 * Constructor.
	 * @param si 			The OMPL state space to use.
	 * @param robot 		The MoveIt robot model.
	 * @param leaves 		The LeavesCollisionChecker to use.
	 */
	LeavesCollisionCountObjective(const ompl::base::SpaceInformationPtr &si,
								  const std::shared_ptr<moveit::core::RobotModel> &robot,
								  const std::shared_ptr<LeavesCollisionChecker> &leaves);

	/**
	 * Compute the cost of a state (which is then used as part of a cost integral operation)
	 *
	 * @param s 			The state to compute the cost of.
	 * @return 				The cost of the state.
	 */
	ompl::base::Cost stateCost(const ompl::base::State *s) const override;

};

#endif //NEW_PLANNERS_LEAVESCOLLISIONCHECKER_H
