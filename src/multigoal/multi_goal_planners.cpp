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
//
void extendTrajectory(robowflex::Trajectory &full_trajectory, const robowflex::Trajectory &extension) {
    // Just loop over all waypoints and copy them over.
    for (size_t i = 0; i < extension.getNumWaypoints(); i++) {
        full_trajectory.addSuffixWaypoint(extension.getTrajectoryConst()->getWayPoint(i));
    }
}
//
//double MultiGoalPlanResult::computeTotalLength() {
//    double total = 0.0;
//    for (const auto &item: segments) {
//        total += item.solution_length;
//    }
//    return total;
//}
//
