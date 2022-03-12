
#include <ompl/geometric/planners/prm/PRMstar.h>
#include "../src/experiment_utils.h"
#include "../src/probe_retreat_move.h"
#include "../src/ManipulatorDroneMoveitPathLengthObjective.h"
#include "../src/traveling_salesman.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/conversions.h>

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
    auto planner = std::make_shared<ompl::geometric::PRMstar>(si);
    auto objective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);

    ompl::base::ScopedState start(si);
    state_space->copyToOMPLState(start.get(), start_state);

    Eigen::Vector3d sphere_center(0.0,0.0,2.0);

    auto result_path = plan_probe_retreat_slide(
            apples_in_order,
            start.get(),
            si,
            [&](const Apple& a, ompl::base::State *result) {
                state_space->copyToOMPLState(result, state_outside_tree(drone, a, sphere_center));
            },
            [&](ompl::base::State *a, ompl::base::State *b) -> std::optional<ompl::geometric::PathGeometric> {
                return planFromStateToState(*planner, objective, a, b, 2.0);
            },
            [&](ompl::base::State *a, const Apple& apple) -> std::optional<ompl::geometric::PathGeometric> {
                return planFromStateToApple(*planner, objective, a, apple, 2.0);
            });

    result_path.interpolate();

    auto moveit_trajectory = omplPathToRobotTrajectory(drone, state_space, result_path);

    ros::init(argc,argv,"probe_retreat_move");

    ros::NodeHandle nh;


    moveit_msgs::RobotTrajectory trasj_msg;

    moveit_msgs::DisplayTrajectory msg;
    msg.model_id = drone->getName();
    msg.trajectory.resize(1);
    moveit_trajectory.getRobotTrajectoryMsg(msg.trajectory[0]);
    moveit::core::robotStateToRobotStateMsg(moveit_trajectory.getFirstWayPoint(), msg.trajectory_start);

    auto traj = nh.advertise<moveit_msgs::DisplayTrajectory>("/trajectory_thingy", 1, true);
    traj.publish(msg);

    auto scene = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1, true);
    scene.publish(scene_msg);

    std::cout << "ready" << std::endl;

    ros::spin();

}

