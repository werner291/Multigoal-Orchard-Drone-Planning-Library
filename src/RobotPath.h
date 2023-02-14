#ifndef NEW_PLANNERS_ROBOTPATH_H
#define NEW_PLANNERS_ROBOTPATH_H

#include <vector>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <ompl/geometric/PathGeometric.h>

/**
 * A path through space defined by a vector of robot states, without an associated time component.
 */
struct RobotPath {
	std::vector<moveit::core::RobotState> waypoints;

	[[nodiscard]] double length() const;

	void append(const RobotPath &other);

	void split_long_segments(double max_segment_length);

	void collapse_short_segments(double min_segment_length);
};

/**
 * Convert an OMPL path to a RobotPath.
 *
 * @param ompl_path 		The OMPL path to convert.
 * @return 					The RobotPath.
 */
RobotPath omplPathToRobotPath(const ompl::geometric::PathGeometric& ompl_path);

/**
 * Convert a RobotPath to an OMPL path.
 *
 * @param robot_path 			The RobotPath to convert.
 * @param si 					The OMPL space information.
 * @return 						The OMPL path.
 */
ompl::geometric::PathGeometric
robotPathToOmplPath(const RobotPath &robot_path, const ompl::base::SpaceInformationPtr &si);

/**
 * Convert a RobotPath to a MoveIt RobotTrajectory.
 *
 * @param robot_path 			The RobotPath to convert.
 * @return 						The RobotTrajectory.
 */
robot_trajectory::RobotTrajectory
robotPathToConstantSpeedRobotTrajectory(const RobotPath &robot_path, const double speed);

#endif //NEW_PLANNERS_ROBOTPATH_H
