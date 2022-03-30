
#ifndef NEW_PLANNERS_ROS_UTILITIES_H
#define NEW_PLANNERS_ROS_UTILITIES_H

#include <ros/publisher.h>
#include <ompl/geometric/PathGeometric.h>
#include "procedural_tree_generation.h"
#include "ompl_custom.h"

/**
 * Publishes a marker array that visualises the given set of approaches using a "pincushion" model.
 *
 * Every apple-approach pair is visualized as a line between the apple and the end-effector of the robot
 * at the first state of the approach trajectory.
 *
 * @param apples A vector of apple-approach pairs to visualise.
 * @param nh The ROS node handle
 * @param topic_name The topic under which to publish this.
 * @param color The color of the markers.
 * @return The publisher... Keep this in scope or the topic may be closed.
 */
[[nodiscard]] ros::Publisher dumpProjections(
        const std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> &apples,
        ros::NodeHandle &nh,
        const std::string &topic_name,
        std_msgs::ColorRGBA color);

/**
 *
 * @param drone
 * @param state_space
 * @param si
 * @param approaches
 * @param nh
 * @param topic_name
 * @return
 */
ros::Publisher dumpApproaches(const moveit::core::RobotModelPtr &drone,
                              const std::shared_ptr<DroneStateSpace> &state_space,
                              const std::shared_ptr<ompl::base::SpaceInformation> &si,
                              const std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> &approaches,
                              ros::NodeHandle &nh,
                              const std::string &topic_name);


#endif //NEW_PLANNERS_ROS_UTILITIES_H
