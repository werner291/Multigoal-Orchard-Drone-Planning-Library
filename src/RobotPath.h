#ifndef NEW_PLANNERS_ROBOTPATH_H
#define NEW_PLANNERS_ROBOTPATH_H

#include <vector>
#include <moveit/robot_state/robot_state.h>
#include <ompl/geometric/PathGeometric.h>

#include "abstract/path_traits.h"

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

// Implement the path_traits for RobotPath.

template<>
struct path_traits<RobotPath> {

	using state_type = moveit::core::RobotState;

	/**
	 * @brief Concatenates two paths into one path.
	 *
	 * @param path1 	The first path.
	 * @param path2 	The second path.
	 *
	 * @return The concatenated path.
	 */
	static RobotPath concatenate_path(const RobotPath &path1, const RobotPath &path2) {
		RobotPath result = path1;
		result.append(path2);
		return result;
	}

	/**
	 * @brief Returns the initial state of the path.
	 *
	 * @param path 	The path.
	 *
	 * @return The initial state of the path.
	 */
	static state_type initial_state(const RobotPath &path) {
		return path.waypoints.front();
	}

	/**
	 * @brief Returns the final state of the path.
	 *
	 * @param path 	The path.
	 *
	 * @return The final state of the path.
	 */
	static state_type final_state(const RobotPath &path) {
		return path.waypoints.back();
	}

};


#endif //NEW_PLANNERS_ROBOTPATH_H
