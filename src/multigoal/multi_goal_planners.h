
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

/**
 * Utility function to extend one trajectory with the waypoints of another.
 *
 * @param full_trajectory Trajectory to be extended.
 * @param extension The trajectory to extend it with.
 */
void extendTrajectory(robowflex::Trajectory &full_trajectory, robowflex::Trajectory &extension);

/**
 * Result struct of a multi-goal planning operation.
 */
struct MultiGoalPlanResult {
    std::vector<PointToPointPlanResult> segments;

    double computeTotalLength();

    std::vector<Eigen::Vector3d> checkMissing(const std::vector<Eigen::Vector3d> &targets);

    robowflex::Trajectory fullTrajectory() {
        robowflex::Trajectory full(segments[0].point_to_point_trajectory.getTrajectory());

        for (int i = 1; i < segments.size(); ++i) {
            extendTrajectory(full, segments[i].point_to_point_trajectory);
        }

        return full;
    }

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
     * @param apples        Information about the scene, including the position of the leaves and the targets.
     * @param start_state   The state of the robot at the start.
     * @param scene         The planning scene, for collision detection.
     * @param robot         The robot to plan for
     * @param point_to_point_planner    The lower-level planner to use when computing a point-tp-point motion.
     * @return              The resulting trajectory along with some statistics.
     */
    virtual MultiGoalPlanResult plan(const TreeScene &apples,
                                     const moveit::core::RobotState &start_state,
                                     const robowflex::SceneConstPtr &scene,
                                     const robowflex::RobotConstPtr &robot,
                                     PointToPointPlanner &point_to_point_planner) = 0;

    /**
     * \brief Returns the name of the meta-planner. Does NOT include any sub-planners or optimization objectives.
     */
    virtual std::string getName() = 0;
};


#endif //MULTI_GOAL_PLANNERS_H
