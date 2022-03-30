#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <range/v3/view/enumerate.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include "ros_utilities.h"
#include "moveit_conversions.h"
#include "msgs_utilities.h"

ros::Publisher
dumpProjections(const std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> &apples,
                ros::NodeHandle &nh,
                const std::string &topic_name, std_msgs::ColorRGBA color) {
    auto sphere_projections = nh.advertise<visualization_msgs::MarkerArray>(topic_name, 1, true);

    visualization_msgs::MarkerArray projections_msg;

    visualization_msgs::Marker balls;
    balls.header.frame_id = "world";
    balls.type = visualization_msgs::Marker::POINTS;
    balls.points.resize(apples.size());
    balls.color = color;
    balls.scale.x = 0.05;
    balls.scale.y = 0.05;
    balls.scale.z = 0.05;
    balls.pose.orientation.w = 1.0;

    visualization_msgs::Marker lines;
    lines.header.frame_id = "world";
    lines.id = 1;
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.scale.x = 0.01;
    lines.color = color;
    lines.pose.orientation.w = 1.0;
    lines.points.resize(2 * apples.size());

    for (const auto &[apple_id, apple]: apples | ranges::views::enumerate) {
        moveit::core::RobotState rs(apple.second.getSpaceInformation()->getStateSpace()->as<DroneStateSpace>()->getRobotModel());

        apple.second.getSpaceInformation()->getStateSpace()->as<DroneStateSpace>()->copyToRobotState(rs, apple.second.getState(0));

        rs.update(true);

        const Eigen::Vector3d onSphere = rs.getGlobalLinkTransform("end_effector").translation();
        tf::pointEigenToMsg(onSphere, balls.points[apple_id]);

        tf::pointEigenToMsg(apple.first.center, lines.points[apple_id * 2]);
        tf::pointEigenToMsg(onSphere, lines.points[apple_id * 2 + 1]);

    }

    projections_msg.markers.push_back(balls);
    projections_msg.markers.push_back(lines);

    sphere_projections.publish(projections_msg);

    return sphere_projections;
}

ros::Publisher dumpApproaches(const moveit::core::RobotModelPtr &drone,
                              const std::shared_ptr<DroneStateSpace> &state_space,
                              const std::shared_ptr<ompl::base::SpaceInformation> &si,
                              const std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> &approaches,
                              ros::NodeHandle &nh,
                              const std::string &topic_name) {

    ompl::geometric::PathGeometric combined_path(si);
    for (const auto &approach: approaches) {
        combined_path.append(approach.second);
    }

    moveit_msgs::DisplayTrajectory msg = robotTrajectoryToDisplayTrajectory(omplPathToRobotTrajectory(
            drone, state_space, combined_path
    ));

    auto traj = nh.advertise<moveit_msgs::DisplayTrajectory>(topic_name, 1, true);

    traj.publish(msg);

    return traj;
}