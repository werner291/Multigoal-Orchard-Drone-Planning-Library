#include <vector>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Core>

/**
 *
 * This function takes a vector of robot states, a vector of index/goal pairs, and attempts to shorten the path
 * by contracting the length of it.
 *
 * @param states
 *
 */
void goal_aware_optimize(std::vector<moveit::core::RobotState> &states,
						 const std::vector<std::pair<int, Eigen::Vector3d>> &ee_goals) {


}