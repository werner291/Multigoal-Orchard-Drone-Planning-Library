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

void extendTrajectory(robowflex::Trajectory &full_trajectory, robowflex::Trajectory &extension) {
    // Just loop over all waypoints and copy them over.
    for (size_t i = 0; i < extension.getNumWaypoints(); i++) {
        full_trajectory.addSuffixWaypoint(extension.getTrajectory()->getWayPoint(i));
    }
}

double MultiGoalPlanResult::computeTotalLength() {
    double total = 0.0;
    for (const auto &item: segments) {
        total += item.solution_length;
    }
    return total;
}

/**
 * \brief Given a vector of targets, returns which have not been visited.
 */
std::vector<Eigen::Vector3d> MultiGoalPlanResult::checkMissing(const std::vector<Eigen::Vector3d> &targets) {

    auto cmp = [](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
        return a.x() != b.x() ? a.x() < b.x() : (a.y() != b.y() ? a.y() < b.y() : a.z() < b.z());
    };

    std::set<Eigen::Vector3d, decltype(cmp)> remaining(targets.begin(), targets.end(), cmp);

    for (const auto &item: segments) {
        remaining.erase(item.endEffectorTarget);
    }

    return std::vector(remaining.begin(), remaining.end());
}
