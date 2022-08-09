
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/terminationconditions/CostConvergenceTerminationCondition.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "../utilities/experiment_utils.h"
#include "../DronePathLengthObjective.h"
#include "../collision_free_shell/SphereShell.h"
#include "../src/ros_utilities.h"

int main(int argc, char **argv) {

    auto drone = loadRobotModel();

    auto[scene_msg, apples, SPHERE_CENTER, SPHERE_RADIUS] = createMeshBasedAppleTreePlanningSceneMessage("appletree");

    std::shuffle( apples.begin(), apples.end(), std::mt19937(std::random_device()()));
//    apples.resize(10);

    int zero = 0;
    ros::init(zero, nullptr, "probe_retreat_move");
    ros::NodeHandle nh;

    auto scene = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1, true);
    scene.publish(scene_msg);

    std::shuffle(apples.begin(), apples.end(), std::mt19937(std::random_device()()));

    auto state_space = std::make_shared<DroneStateSpace>(
            ompl_interface::ModelBasedStateSpaceSpecification(drone, "whole_body"), 5.0);
    auto si = initSpaceInformation(setupPlanningScene(scene_msg, drone), drone, state_space);
    auto objective = std::make_shared<DronePathLengthObjective>(si);

    const SphereShell sphereShell(SPHERE_CENTER, 1.8);

    std::vector<ros::Publisher> publisher_handles;

    OMPLSphereShellWrapper shell(sphereShell, si);

    std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> approaches;

    for (const Apple &apple: apples) {

        auto stateOutside = shell.state_on_shell(apple);

        auto planner = std::make_shared<ompl::geometric::RRTstar>(si);

        ompl::base::State *a = stateOutside->get();
        auto pdef = std::make_shared<ompl::base::ProblemDefinition>((*planner).getSpaceInformation());
        pdef->setOptimizationObjective(objective);
        pdef->addStartState(a);
        pdef->setGoal(
                std::make_shared<DroneEndEffectorNearTarget>((*planner).getSpaceInformation(), 0.05, apple.center));

        (*planner).setProblemDefinition(pdef);

        if ((*planner).solve(ompl::base::timedPlannerTerminationCondition(1.0)) ==
            ompl::base::PlannerStatus::EXACT_SOLUTION) {

            ompl::geometric::PathGeometric path = *pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();

            std::cout << "Approach planning finished: " << path.length() << std::endl;

            path.interpolate();

            approaches.emplace_back(apple, path);
        }
    }

    auto handle = dumpApproaches(drone, state_space, si, approaches, nh, "approaches");

    ros::spin();

}