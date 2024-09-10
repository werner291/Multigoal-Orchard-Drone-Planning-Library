// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/22/24.
//

#ifndef MGODPL_ROBOTPATH_H
#define MGODPL_ROBOTPATH_H


#include <vector>
#include "RobotState.h"
#include "distance.h"

namespace mgodpl {
	/**
	 * @struct RobotPath
	 * @brief A structure representing a path for the robot.
	 *
	 * This structure contains a vector of RobotState objects,
	 * each representing a state of the robot at a certain point along the path.
	 *
	 * The path itself is interpreted as a sequence of
	 * linearly-interpolated motions between the states.
	 */
	struct RobotPath {
		/// @brief A vector of RobotState objects representing the states of the robot along the path.
		std::vector<RobotState> states;

		/**
		 * @brief Appends a new state to the path.
		 * @param state The RobotState to append.
		 */
		void append(const RobotState &state) {
			states.push_back(state);
		}

		/**
		 * @brief Appends another path to this path.
		 * @param path The RobotPath to append.
		 */
		void append(const RobotPath &path) {
			states.insert(states.end(), path.states.begin(), path.states.end());
		}

		/**
		 * @brief Create a singleton path with a single state.
		 * @param state The state to create the path with.
		 * @return A new RobotPath with the given state.
		 */
		static RobotPath singleton(RobotState state) {
			return {
					.states = {state}
			};
		}

		/**
		 * @brief Get the number of waypoints in the path.
		 * @return The number of waypoints.
		 */
		[[nodiscard]] size_t n_waypoints() const {
			return states.size();
		}

		/**
		 * @brief Check if the path is empty.
		 * @return True if the path is empty, false otherwise.
		 */
		[[nodiscard]] bool empty() const {
			return states.empty();
		}

		/**
		 * @brief Get the starting state of the path.
		 * @return The starting RobotState.
		 */
		[[nodiscard]] const RobotState &start() const {
			return states.front();
		}

		/**
		 * @brief Get the ending state of the path.
		 * @return The ending RobotState.
		 */
		[[nodiscard]] const RobotState &end() const {
			return states.back();
		}

		/**
		 * @brief Get the state at a specific waypoint.
		 * @param i The index of the waypoint.
		 * @return The RobotState at the specified waypoint.
		 */
		[[nodiscard]] const RobotState &waypoint(size_t i) const {
			return states.at(i);
		}

		/**
		 * @brief Check if the path is a singleton (contains only one state).
		 * @return True if the path is a singleton, false otherwise.
		 */
		[[nodiscard]] bool is_singleton() const {
			return states.size() == 1;
		}
	};

	/**
	 * @struct PathPoint
	 * @brief A structure representing a point on a path.
	 *
	 * This structure is used to define a point on a path based on states and the interpolated motions between them.
	 */
	struct PathPoint {
		/// @brief The index of the segment on the path. This is the segment between states segment_i and segment_i+1 (0-indexed).
		size_t segment_i;

		/// @brief The time at which the robot is at this point on the segment.
		double segment_t;

		/**
		 * @brief Converts a scalar value to a PathPoint.
		 *
		 * This function takes a scalar value and a RobotPath as input and returns a PathPoint, using the integer part of
		 * the scalar value as the segment index and the fractional part as the time along the segment.
		 *
		 * The segment index is clamped to the range [0, path.states.size() - 2].
		 *
		 * @param d The scalar value to convert to a PathPoint.
		 * @param path The RobotPath to use for determining the segment index.
		 * @return A PathPoint with the segment index and time determined from the scalar value and the RobotPath.
		 */
		static PathPoint fromScalar(double d, const mgodpl::RobotPath &path) {
			PathPoint path_point;
			path_point.segment_i = (size_t) std::floor(d);
			path_point.segment_i = std::min(path_point.segment_i, path.states.size() - 2);
			path_point.segment_t = d - (double) path_point.segment_i;
			return path_point;
		}

		/**
		 * @brief Converts a PathPoint to a scalar value in the range [0, path.states.size() - 1).
		 * @return The scalar value corresponding to this PathPoint.
		 */
		[[nodiscard]] double toScalar() const {
			return (double) segment_i + segment_t;
		}

		/**
		 * @brief Adjust the PathPoint by a scalar value, clamping to the range [0, path.states.size() - 1).
		 * @param d 		The scalar value to adjust by.
		 * @param path 		The RobotPath to use for determining the segment index.
		 * @return 			The adjusted PathPoint.
		 */
		[[nodiscard]] PathPoint adjustByScalar(double d, const mgodpl::RobotPath &path) const {
			return fromScalar(toScalar() + d, path);
		}

		/**
		 * @brief			Compare two PathPoints.
		 * @param other		The other PathPoint to compare to.
		 * @return			True if this PathPoint is less than the other, false otherwise.
		 */
		[[nodiscard]] bool operator<(const PathPoint &other) const {
			return toScalar() < other.toScalar();
		}

		/**
		 * @brief			Compare two PathPoints.
		 * @param other		The other PathPoint to compare to.
		 * @return			True if this PathPoint is after than the other, false otherwise.
		 */
		[[nodiscard]] bool operator>(const PathPoint &other) const {
			return toScalar() > other.toScalar();
		}
	};

	/**
	 * @brief Interpolates between two states of a robot path at a given point.
	 *
	 * @param path_point The point on the path where the interpolation is to be done.
	 * @param robot_path The path of the robot.
	 * @return The interpolated RobotState at the given point on the path.
	 */
	RobotState interpolate(const mgodpl::PathPoint &path_point, const mgodpl::RobotPath &robot_path);

	/**
	 * @brief Calculate the length of a segment in a robot path.
	 *
	 * TODO: Using PathPoint for this is really weird, do we want to keep it like this?
	 *
	 * This function calculates the length of the segment at the given point on the path.
	 * The length is calculated using the provided distance function.
	 *
	 * @param robot_path The robot path.
	 * @param path_point The point on the path that falls into the segment.
	 * @param distanceFunc The function to use for calculating the distance between two states.
	 * @return The length of the segment.
	 */
	double calculateSegmentLength(const mgodpl::RobotPath &robot_path,
								  const mgodpl::PathPoint &path_point,
								  DistanceFn distanceFunc);

	/**
	 * @brief Clamp a path point to the end of a robot path.
	 *
	 * This function modifies a path point so that it does not exceed the end of the robot path.
	 * If the path point is at or beyond the end of the path, it is set to the last state of the path.
	 *
	 * @param robot_path The robot path.
	 * @param path_point The path point to clamp (this is an output parameter).
	 *
	 * @return  True if the path point was clamped, and false otherwise.
	 */
	bool clampPathPoint(const mgodpl::RobotPath &robot_path, mgodpl::PathPoint &path_point);

	/**
	 * @brief Wrap a path point to the start of a robot path.
	 *
	 * This function modifies a path point so that it wraps around to the start of the robot path
	 * when it reaches the end of the path.
	 *
	 * @param robot_path The robot path.
	 * @param path_point The path point to wrap (this is an output parameter).
	 *
	 * @return True if the path point was wrapped, and false otherwise.
	 */
	bool wrapPathPoint(const mgodpl::RobotPath &robot_path, mgodpl::PathPoint &path_point);

	/**
	 * @brief Advance a path point along a robot path, clamping at the end.
	 *
	 * This function advances a path point along a robot path by a given amount.
	 * The path point is clamped to the end of the path if it would otherwise exceed it.
	 * The function returns true if the path point was clamped, and false otherwise.
	 *
	 * @param robot_path The robot path.
	 * @param path_point The path point to advance.
	 * @param advancement The amount to advance the path point.
	 * @param distanceFunc The function to use for calculating the distance between two states.
	 * @return True if the path point was clamped, false otherwise.
	 */
	bool advancePathPointClamp(const mgodpl::RobotPath &robot_path,
							   mgodpl::PathPoint &path_point,
							   double advancement,
							   DistanceFn distanceFunc);

	/**
	 * @brief Advance a path point along a robot path, wrapping at the end.
	 *
	 * This function advances a path point along a robot path by a given amount.
	 * The path point wraps around to the start of the path if it reaches the end of the path.
	 * The function returns true if the path point was wrapped, and false otherwise.
	 *
	 * @param robot_path The robot path.
	 * @param path_point The path point to advance.
	 * @param advancement The amount to advance the path point.
	 * @param distanceFunc The function to use for calculating the distance between two states.
	 * @return True if the path point was wrapped, false otherwise.
	 */
	bool advancePathPointWrap(const mgodpl::RobotPath &robot_path,
							  mgodpl::PathPoint &path_point,
							  double advancement,
							  DistanceFn distanceFunc);

	/**
	 * @brief Concatenates two RobotPaths.
	 *
	 * This function takes two RobotPath objects as input and returns a new RobotPath that is the concatenation of the two input paths.
	 * The states of the second path are appended to the states of the first path in the resulting RobotPath.
	 *
	 * @param path1 The first RobotPath.
	 * @param path2 The second RobotPath.
	 * @return A new RobotPath that is the concatenation of path1 and path2.
	 */
	RobotPath concatenate(const RobotPath &path1, const RobotPath &path2);

	/**
	 * @brief Reverses a RobotPath.
	 *
	 * This function takes a RobotPath object as input and returns a new RobotPath that is the reverse of the input path.
	 * The states in the resulting RobotPath are in the reverse order of the states in the input path.
	 *
	 * @param path The RobotPath to reverse.
	 * @return A new RobotPath that is the reverse of the input path.
	 */
	RobotPath reverse(const RobotPath &path);

	/**
	 * Modify a path, inserting the given number of sub-steps into each segment.
	 *
	 * @param num_steps 		The number of sub-steps to insert into each segment.
	 * @return 				A new path with the sub-steps inserted.
	 */
	RobotPath subdivided(const RobotPath &original, size_t num_steps);

	/**
	 * @brief Compute the path length using a given inter-state distance function.
	 *
	 * @param path 			The path to compute the length of.
	 * @param distanceFunc 	The inter-state distance function to use (default is equal_weights_distance).
	 * @return The length of the path.
	 */
	double pathLength(const RobotPath &path, const DistanceFn &distanceFunc = equal_weights_distance);

	/**
	 * @brief Compute the roughness of the path using a given roughness function, summing the roughness of every waypoint.
	 *
	 * @param path 			The path to compute the roughness of.
	 * @param roughnessFunc The roughness function to use, taking a triple of states and returning a double.
	 *
	 * @return The roughness of the path (lower is smoother).
	 */
	double pathRoughness(const RobotPath &path,
						 const std::function<double(const RobotState &,
													const RobotState &,
													const RobotState &)> &roughnessFunc);
}


#endif //MGODPL_ROBOTPATH_H
