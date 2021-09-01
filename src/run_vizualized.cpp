#include <robowflex_library/builder.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_library/util.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/io/broadcaster.h>
#include "build_request.h"
#include "build_planning_scene.h"
#include "make_robot.h"
#include "init_planner.h"
#include "InverseClearanceIntegralObjective.h"
#include "ClearanceDecreaseMinimizationObjective.h"
#include <fcl/fcl.h>

using namespace robowflex;

/**
 * The "visualized" version of this program, which serves as a scratch state in which to experiment with new,
 * and potentially useless changes.
 *
 * See the benchmark main() method for the more reproducible results.
 */
int main(int argc, char **argv) {

    // Startup ROS
    ROS ros(argc, argv);

    std::shared_ptr<Robot> drone = make_robot();

    IO::RVIZHelper rviz(drone);
    IO::RobotBroadcaster bc(drone);
    bc.start();

    auto scene = std::make_shared<Scene>(drone);
    auto tree_scene = establishPlanningScene(10);
    scene->getScene()->setPlanningSceneDiffMsg(tree_scene.moveit_diff);

    rviz.updateScene(scene);

    ompl::msg::setLogLevel(ompl::msg::LOG_INFO);

    auto optimizationObjectiveAllocator = [](const ompl::geometric::SimpleSetupPtr &ss) {
        return std::make_shared<ClearanceDecreaseMinimizationObjective>(ss->getSpaceInformation());
    };

    moveit_msgs::MotionPlanRequest request = makeAppleReachRequest(drone, "RRTConnect", 60.0,
                                                                   selectAppleNearCoG(tree_scene.apples));
    rviz.addGoalMarker("goal_request_marker", request);
    rviz.updateMarkers();
    auto simple_planner = init_planner(drone, scene, optimizationObjectiveAllocator);
    auto response = simple_planner->plan(scene, request);
    if (response.error_code_.val == 1) {
        rviz.updateTrajectory(response);
    }

    std::cin.get();

    return 0;
}


