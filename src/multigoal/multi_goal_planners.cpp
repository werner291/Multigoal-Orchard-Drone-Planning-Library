#include <ompl/geometric/planners/prm/PRM.h>
#include <robowflex_library/trajectory.h>

#include <random>
#include "multi_goal_planners.h"

void extendTrajectory(robowflex::Trajectory &full_trajectory, const robowflex::Trajectory &extension) {
    // Just loop over all waypoints and copy them over.
    for (size_t i = 0; i < extension.getNumWaypoints(); i++) {
        full_trajectory.addSuffixWaypoint(extension.getTrajectoryConst()->getWayPoint(i));
    }
}