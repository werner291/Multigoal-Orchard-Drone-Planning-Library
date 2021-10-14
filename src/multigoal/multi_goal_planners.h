
#ifndef MULTI_GOAL_PLANNERS_H
#define MULTI_GOAL_PLANNERS_H

#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <robowflex_ompl/ompl_interface.h>
#include <robowflex_library/trajectory.h>
#include <json/value.h>
#include "../procedural_tree_generation.h"
#include "../ompl_custom.h"
#include "../LeavesCollisionChecker.h"
#include "PointToPointPlanner.h"

// For optimizing planners, approximately how much time should be left to the planning operation per target, all other things equal.
const double MAX_TIME_PER_TARGET_SECONDS = 0.1;

typedef std::shared_ptr<ompl::base::GoalSampleableRegion> GoalSamplerPtr;

/**
 * Utility function to extend one trajectory with the waypoints of another.
 *
 * @param full_trajectory Trajectory to be extended.
 * @param extension The trajectory to extend it with.
 */
void extendTrajectory(robowflex::Trajectory &full_trajectory, const robowflex::Trajectory &extension);

struct PointToPointPath {
    size_t to_goal;
    ompl::geometric::PathGeometric path;
};

/**
 * Result struct of a multi-goal planning operation.
 */
struct MultiGoalPlanResult {
    std::vector<PointToPointPath> segments;
};


/**
 * A meta-planner that, given a number of targets to visit,
 * attempts to plan a trajectory that brings the end-effector
 * of the robot close to each target.
 */
class MultiGoalPlanner {

public:

    /**
     * Plan the trajectory.
     *
     * @param goals                  List of GoalSampleableRegion, the planner will attempt to visit all.
     * @param start_state            The state of the robot at the start.
     * @param point_to_point_planner Wrapper for an OMPL planner and an optimization objective
     */
    virtual MultiGoalPlanResult plan(const std::vector<GoalSamplerPtr> &goals,
                                     const ompl::base::State *start_state,
                                     PointToPointPlanner &point_to_point_planner) = 0;

    /**
     * \brief Returns the name of the meta-planner. Does NOT include any sub-planners or optimization objectives.
     */
    virtual std::string getName() = 0;
};

#endif //MULTI_GOAL_PLANNERS_H
