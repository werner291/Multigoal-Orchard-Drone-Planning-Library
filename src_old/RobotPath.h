#ifndef NEW_PLANNERS_ROBOTPATH_H
#define NEW_PLANNERS_ROBOTPATH_H

#include <vector>
#include <ompl/geometric/PathGeometric.h>
#include "PathInterrupt.h"

// Moveit pulls in Eigen/Geometry, which is a massive dependency.
// Instead, we forward declare the RobotState class and include the header in RobotPath.cpp.
namespace moveit::core {
	class RobotState;
}

namespace robot_trajectory {
	class RobotTrajectory;
}

/**
 * A path through space defined by a vector of robot states, without an associated time component.
 */
struct RobotPath {

	using Configuration = moveit::core::RobotState;

	std::vector<moveit::core::RobotState> waypoints;

	[[nodiscard]] double length() const;

	void append(const RobotPath &other);

	void split_long_segments(double max_segment_length);

	void collapse_short_segments(double min_segment_length);

	/**
	 * Truncate the path up to the given interrupt.
	 *
	 * That is, if the path is [0 1 2 3 4 5 6 7 8 9] and the interrupt is at 5.3, the path will be truncated
	 * to [0 1 2 3 4 5 5.3], interpolating the final waypoint as needed.
	 *
	 * @param interrupt 		The interrupt.
	 */
	void truncateUpTo(PathInterrupt interrupt);

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
