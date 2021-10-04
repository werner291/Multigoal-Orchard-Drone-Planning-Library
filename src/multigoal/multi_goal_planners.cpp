#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <robowflex_library/trajectory.h>

#include <random>
#include "multi_goal_planners.h"
#include "../ompl_custom.h"
#include "../procedural_tree_generation.h"
#include "../json_utils.h"
#include "../UnionGoalSampleableRegion.h"
#include "../LeavesCollisionChecker.h"
#include "knn.h"
#include "uknn.h"
#include "random_order.h"
#include "PointToPointPlanner.h"

std::shared_ptr<ompl::base::SpaceInformation>
initSpaceInformation(const robowflex::SceneConstPtr &scene, const robowflex::RobotConstPtr &robot,
                     std::shared_ptr<DroneStateSpace> &state_space);


void extendTrajectory(robowflex::Trajectory &full_trajectory, robowflex::Trajectory &extension) {
    for (size_t i = 0; i < extension.getNumWaypoints(); i++) {
        full_trajectory.addSuffixWaypoint(extension.getTrajectory()->getWayPoint(i));
    }
}