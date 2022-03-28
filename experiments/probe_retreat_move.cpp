
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

int main(int argc, char** argv) {

    auto drone = loadRobotModel();

    auto[scene_msg, apples] = createMeshBasedAppleTreePlanningSceneMessage();

    moveit::core::RobotState start_state = stateOutsideTree(drone);

    std::shuffle(apples.begin(), apples.end(), std::mt19937(std::random_device()()));
    apples.resize(10);

    const Eigen::Vector3d sphere_center(0.0,0.0,2.2);

    std::vector<Apple> apples_in_order = vectorByOrdering(apples, ORToolsOrderingStrategy().apple_ordering(apples, GreatcircleDistanceHeuristics(
            start_state.getGlobalLinkTransform("end_effector").translation(),
            GreatCircleMetric(sphere_center)
    )));

    auto state_space = std::make_shared<DroneStateSpace>(ompl_interface::ModelBasedStateSpaceSpecification(drone, "whole_body"));
    auto si = initSpaceInformation(setupPlanningScene(scene_msg, drone), drone, state_space);
    auto objective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);

    ompl::base::ScopedState start(si);
    state_space->copyToOMPLState(start.get(), start_state);

    const SphereShell sphereShell(sphere_center, 4);

    OMPLSphereShellWrapper shell(sphereShell, si);
    const auto approaches = planApproaches(apples_in_order, objective, shell, si);
    auto fullPath = planFullPath(si, start.get(), shell, approaches);
    auto moveit_trajectory = omplPathToRobotTrajectory(drone, state_space, fullPath);

    int zero = 0;
    ros::init(zero, nullptr, "probe_retreat_move");

    ros::NodeHandle nh;

    moveit_msgs::DisplayTrajectory msg = robotTrajectoryToDisplayTrajectory(moveit_trajectory);

    auto traj = nh.advertise<moveit_msgs::DisplayTrajectory>("/trajectory_thingy", 1, true);
    traj.publish(msg);

    auto sphere_projections = nh.advertise<visualization_msgs::MarkerArray>("/apple_to_sphere_projections", 1, true);
    {
        visualization_msgs::MarkerArray projections_msg;

        visualization_msgs::Marker balls;
        balls.type = visualization_msgs::Marker::POINTS;
        balls.points.resize(apples.size());

        visualization_msgs::Marker lines;
        lines.type = visualization_msgs::Marker::LINE_LIST;
        lines.points.resize(2*apples.size());

        for (const auto &[apple_id, apple] : apples | ranges::views::enumerate) {

            const Eigen::Vector3d &onSphere = sphereShell.applePositionOnSphere(apple);
            tf::pointEigenToMsg(onSphere,balls.points[apple_id]);

            tf::pointEigenToMsg(apple.center,lines.points[apple_id*2]);
            tf::pointEigenToMsg(onSphere,lines.points[apple_id*2+1]);

        }

        projections_msg.markers.push_back(balls);
        projections_msg.markers.push_back(lines);

        traj.publish(projections_msg);
    }

    auto scene = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1, true);
    scene.publish(scene_msg);

    std::cout << "ready" << std::endl;

    ros::spin();

}

