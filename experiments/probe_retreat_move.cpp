
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/conversions.h>

#include "../src/experiment_utils.h"
#include "../src/probe_retreat_move.h"
#include "../src/ManipulatorDroneMoveitPathLengthObjective.h"
#include "../src/traveling_salesman.h"
#include "../src/general_utilities.h"
#include "../src/msgs_utilities.h"

#include <range/v3/all.hpp>
#include <boost/format.hpp>
#include <eigen_conversions/eigen_msg.h>

void dumpApproaches(const moveit::core::RobotModelPtr &drone, const std::shared_ptr<DroneStateSpace> &state_space,
                    const std::shared_ptr<ompl::base::SpaceInformation> &si,
                    const std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> &approaches,
                    ros::NodeHandle &nh, const std::string &topic_name);

ros::Publisher dumpProjections(const std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> &apples,
                               const SphereShell &sphereShell,
                               ros::NodeHandle &nh,
                               const std::string &topic_name,
                               std_msgs::ColorRGBA color) {
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

int main(int argc, char **argv) {

    auto drone = loadRobotModel();

    auto[scene_msg, apples] = createMeshBasedAppleTreePlanningSceneMessage();

    int zero = 0;
    ros::init(zero, nullptr, "probe_retreat_move");
    ros::NodeHandle nh;

    auto scene = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1, true);
    scene.publish(scene_msg);

    moveit::core::RobotState start_state = stateOutsideTree(drone);

    std::shuffle(apples.begin(), apples.end(), std::mt19937(std::random_device()()));
//    apples.resize(10);

    GreatcircleDistanceHeuristics gdh(start_state.getGlobalLinkTransform("end_effector").translation(),
            GreatCircleMetric(SPHERE_CENTER));

    auto state_space = std::make_shared<DroneStateSpace>(
            ompl_interface::ModelBasedStateSpaceSpecification(drone, "whole_body"));
    auto si = initSpaceInformation(setupPlanningScene(scene_msg, drone), drone, state_space);
    auto objective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);

    ompl::base::ScopedState start(si);
    state_space->copyToOMPLState(start.get(), start_state);

    const SphereShell sphereShell(SPHERE_CENTER, 1.8);

    std::vector<ros::Publisher> publisher_handles;

    OMPLSphereShellWrapper shell(sphereShell, si);

    const std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> approaches_naive = planApproaches(apples, objective, shell, si);

    std::cout << "Approaches planned..." << std::endl;

    const std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> approaches_optimized = approaches_naive | ranges::views::transform([&](const auto pair) {
        ompl::geometric::PathGeometric path_copy = pair.second;
        optimizeExit(pair.first, path_copy, objective, shell, si);
        return std::make_pair(pair.first, path_copy);
    }) | ranges::to_vector;

    std_msgs::ColorRGBA green; green.r = 0.0; green.g = 1.0; green.b = 0.0; green.a = 1.0;
    std_msgs::ColorRGBA white; white.r = 1.0; white.g = 1.0; white.b = 1.0; white.a = 1.0;

    std::tuple<std::string, std::vector<std::pair<Apple, ompl::geometric::PathGeometric>>, std_msgs::ColorRGBA> orderings[] = {
            {"naive",     approaches_naive, green },
            {"optimized", approaches_optimized, white}
    };

    for (const auto &[approach_type, approaches, color] : orderings) {
        dumpApproaches(drone, state_space, si, approaches, nh, "/approaches_"+approach_type);

        auto apples_after_approach = approaches | ranges::views::transform([](auto pair) {return pair.first;}) | ranges::to_vector;

        auto approaches_by_gcd = vectorByOrdering(approaches , ORToolsOrderingStrategy().apple_ordering(apples_after_approach, gdh));

        publisher_handles.push_back(dumpProjections(approaches, sphereShell, nh, "/apple_to_sphere_projections_"+approach_type, color));

        auto fullPath = planFullPath(si, start.get(), shell, approaches_by_gcd);
        auto moveit_trajectory = omplPathToRobotTrajectory(drone, state_space, fullPath);
        moveit_msgs::DisplayTrajectory msg = robotTrajectoryToDisplayTrajectory(moveit_trajectory);
        auto traj = nh.advertise<moveit_msgs::DisplayTrajectory>("/trajectory_"+approach_type, 1, true);
        traj.publish(msg);
    }

    std::cout << "ready" << std::endl;

    ros::spin();

}



void dumpApproaches(const moveit::core::RobotModelPtr &drone, const std::shared_ptr<DroneStateSpace> &state_space,
                    const std::shared_ptr<ompl::base::SpaceInformation> &si,
                    const std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> &approaches,
                    ros::NodeHandle &nh, const std::string &topic_name) {

    ompl::geometric::PathGeometric combined_path(si);
    for (const auto &approach: approaches) {
        combined_path.append(approach.second);
    }

    moveit_msgs::DisplayTrajectory msg = robotTrajectoryToDisplayTrajectory(omplPathToRobotTrajectory(
            drone, state_space, combined_path
    ));

    auto traj = nh.advertise<moveit_msgs::DisplayTrajectory>(topic_name, 1, true);

    traj.publish(msg);
}

