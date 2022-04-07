#include <ros/init.h>
#include <moveit_msgs/PlanningScene.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include "ExperimentVisualTools.h"
#include "procedural_tree_generation.h"
#include "probe_retreat_move.h"
#include "msgs_utilities.h"

/// Alternative to ros::init that does not take a mutable reference to an int.
void rosinit_noparam() {
    int zero = 0;
    ros::init(zero, nullptr, "probe_retreat_move");
}

ExperimentVisualTools::ExperimentVisualTools() : nh((rosinit_noparam(),"evt")) {

}

void ExperimentVisualTools::publishPlanningScene(const moveit_msgs::PlanningScene &scene_msg) {
    auto scene = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1, true);
    scene.publish(scene_msg);
    publisher_handles.push_back(scene);
}

void ExperimentVisualTools::dumpProjections(const std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> &apples,
                                            const std::string &topic_name) {

    std_msgs::ColorRGBA color;

    ompl::RNG rng;
    color.a = 1.0;
    color.r = (float) rng.uniform01();
    color.g = (float) rng.uniform01();
    color.b = (float) rng.uniform01();

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

        const Eigen::Vector3d onSphere = appleFromApproach(
                *apple.second.getSpaceInformation()->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>(),
                apple.second).center;

        tf::pointEigenToMsg(onSphere, balls.points[apple_id]);

        tf::pointEigenToMsg(apple.first.center, lines.points[apple_id * 2]);
        tf::pointEigenToMsg(onSphere, lines.points[apple_id * 2 + 1]);

    }

    projections_msg.markers.push_back(balls);
    projections_msg.markers.push_back(lines);

    sphere_projections.publish(projections_msg);

    publisher_handles.push_back(sphere_projections);
}

void ExperimentVisualTools::dumpApproaches(const std::shared_ptr<ompl::base::SpaceInformation> &si,
                                           const std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> &approaches,
                                           const std::string &topic_name) {

    ompl::geometric::PathGeometric combined_path(si);

    for (const auto &approach: approaches) {
        combined_path.append(approach.second);
    }

    publishPath(si, topic_name, combined_path);

}

void ExperimentVisualTools::publishPath(const std::shared_ptr<ompl::base::SpaceInformation> &si,
                                        const std::string &topic_name,
                                        const ompl::geometric::PathGeometric &combined_path) {
    auto moveit_trajectory = omplPathToRobotTrajectory(
            *si->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>(),
            combined_path
            );

    moveit_msgs::DisplayTrajectory msg = robotTrajectoryToDisplayTrajectory(moveit_trajectory);

    auto traj = nh.advertise<moveit_msgs::DisplayTrajectory>(topic_name, 1, true);

    traj.publish(msg);

    publisher_handles.push_back(traj);
}
