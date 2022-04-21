
#ifndef NEW_PLANNERS_EXPERIMENTVISUALTOOLS_H
#define NEW_PLANNERS_EXPERIMENTVISUALTOOLS_H

#include <ros/node_handle.h>
#include <range/v3/view/enumerate.hpp>
#include <ompl/geometric/PathGeometric.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "procedural_tree_generation.h"


class ExperimentVisualTools {

    ros::NodeHandle nh;
    std::vector<ros::Publisher> publisher_handles;

public:
    ExperimentVisualTools();

    void publishPlanningScene(const moveit_msgs::PlanningScene& scene_msg);

    void dumpProjections(const std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> &apples,
                         const std::string &topic_name);

    void dumpApproaches(const std::shared_ptr<ompl::base::SpaceInformation> &si,
                        const std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> &approaches,
                        const std::string &topic_name);

    void publishPath(const std::shared_ptr<ompl::base::SpaceInformation> &si, const std::string &topic_name,
                     const ompl::geometric::PathGeometric &combined_path);

    void publishTrajectory(const std::string &topic_name, const robot_trajectory::RobotTrajectory &moveit_trajectory);
};

#endif //NEW_PLANNERS_EXPERIMENTVISUALTOOLS_H
