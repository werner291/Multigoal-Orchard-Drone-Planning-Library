
#include <ompl/geometric/planners/prm/PRMstar.h>
#include "../src/experiment_utils.h"
#include "../src/probe_retreat_move.h"
#include "../src/ManipulatorDroneMoveitPathLengthObjective.h"
#include "../src/traveling_salesman.h"
#include "../src/SphereShell.h"
#include "../src/moveit_conversions.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/conversions.h>
#include <ompl/geometric/PathSimplifier.h>

robot_trajectory::RobotTrajectory
omplPathToRobotTrajectory(const moveit::core::RobotModelPtr &drone,
                          const std::shared_ptr<DroneStateSpace> &state_space,
                          ompl::geometric::PathGeometric &result_path) {

    robot_trajectory::RobotTrajectory traj(drone);

    double t = 0.0;
    for (const auto state : result_path.getStates()) {
        moveit::core::RobotState moveit_state(drone);
        state_space->copyToRobotState(moveit_state, state);
        traj.addSuffixWayPoint(moveit_state, t);
        t += 0.1;
    }

    return traj;
}


moveit_msgs::DisplayTrajectory
robotTrajectoryToDisplayTrajectory(const robot_trajectory::RobotTrajectory &moveit_trajectory) {
    moveit_msgs::DisplayTrajectory msg;
    msg.model_id = moveit_trajectory.getRobotModel()->getName();
    msg.trajectory.resize(1);
    moveit_trajectory.getRobotTrajectoryMsg(msg.trajectory[0]);
    moveit::core::robotStateToRobotStateMsg(moveit_trajectory.getFirstWayPoint(), msg.trajectory_start);
    return msg;
}

int main(int argc, char** argv) {

    auto drone = loadRobotModel();

    auto[scene_msg, apples] = createMeshBasedAppleTreePlanningSceneMessage();

    std::shuffle(apples.begin(), apples.end(), std::mt19937(std::random_device()()));
    apples.resize(10);

    moveit::core::RobotState start_state(drone);

    start_state.setVariablePositions({
                                             3.0, 3.0, 1.5,      // Position off the side of the tree
                                             0.0, 0.0, 0.0, 1.0, // Identity rotation
                                             0.0, 0.0, 0.0, 0.0  // Arm straight out
                                     });

    start_state.update(true);

    auto ordering = ORToolsOrderingStrategy().apple_ordering(apples, GreatcircleDistanceHeuristics(
        start_state.getGlobalLinkTransform("end_effector").translation(),
        GreatCircleMetric(Eigen::Vector3d(0,0,2.2))
    ));

    std::vector<Apple> apples_in_order; for (const size_t i : ordering) apples_in_order.push_back(apples[i]);

    ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");
    auto state_space = std::make_shared<DroneStateSpace>(spec);
    state_space->setup();
    auto si = initSpaceInformation(setupPlanningScene(scene_msg, drone), drone, state_space);
    // FIXME see https://github.com/ompl/ompl/issues/885 auto planner = std::make_shared<ompl::geometric::AITstar>(si);
    auto objective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);

    ompl::base::ScopedState start(si);
    state_space->copyToOMPLState(start.get(), start_state);

    Eigen::Vector3d sphere_center(0.0,0.0,2.0);

    const ompl::base::State *initialState = start.get();

    SphereShell shell(sphere_center, 4);

    std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> approaches;
    for (const Apple &apple: apples_in_order) {

        ompl::base::ScopedState stateOutsideTreeForApple(si);
        state_space->copyToOMPLState(stateOutsideTreeForApple.get(), shell.state_on_shell(drone, apple));

        auto planner = std::make_shared<ompl::geometric::PRMstar>(si);
        if (auto approach = planFromStateToApple(*planner, objective, stateOutsideTreeForApple.get(), apple, 1.0)) {
            approaches.emplace_back(apple, *approach);
        }
    }

    ompl::geometric::PathGeometric fullPath(si, initialState);

    for (size_t approachIdx: boost::irange<size_t>(1, approaches.size())) {
        ompl::geometric::PathGeometric appleToApple(approaches[approachIdx - 1].second);
        appleToApple.reverse();

        appleToApple.append(
                omplPathFromMoveitTrajectory(shell.path_on_shell(drone,approaches[approachIdx - 1].first,approaches[approachIdx - 1].first),si)
                );

        appleToApple.append(approaches[approachIdx].second);

        ompl::geometric::PathSimplifier(si).simplifyMax(appleToApple);

        fullPath.append(appleToApple);
    }

    fullPath.interpolate();

    auto moveit_trajectory = omplPathToRobotTrajectory(drone, state_space, fullPath);

    ros::init(argc,argv,"probe_retreat_move");

    ros::NodeHandle nh;

    moveit_msgs::DisplayTrajectory msg = robotTrajectoryToDisplayTrajectory(moveit_trajectory);

    auto traj = nh.advertise<moveit_msgs::DisplayTrajectory>("/trajectory_thingy", 1, true);
    traj.publish(msg);

    auto scene = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1, true);
    scene.publish(scene_msg);

    std::cout << "ready" << std::endl;

    ros::spin();

}

