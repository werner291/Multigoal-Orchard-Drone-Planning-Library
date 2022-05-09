#ifndef NEW_PLANNERS_ROBOT_PATH_H
#define NEW_PLANNERS_ROBOT_PATH_H

#include <vector>
#include <moveit/robot_state/robot_state.h>
#include <ompl/geometric/PathGeometric.h>

/**
 * A path through space defined by a vector of robot states, without an associated time component.
 */
struct RobotPath {
    std::vector<moveit::core::RobotState> waypoints;

    [[nodiscard]] double length() const;

    void append(const RobotPath &other);
};

RobotPath omplPathToRobotPath(const ompl::geometric::PathGeometric& ompl_path);

#endif //NEW_PLANNERS_ROBOT_PATH_H
