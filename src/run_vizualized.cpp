#include <robowflex_library/builder.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_library/util.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/io/broadcaster.h>
#include "build_request.h"
#include "build_planning_scene.h"
#include "make_robot.h"
#include "init_planner.h"
#include "MyCollisionDetectorAllocatorBullet.h"

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
    auto tree_scene = establishPlanningScene(8);
    scene->getScene()->setPlanningSceneDiffMsg(tree_scene.moveit_diff);

    rviz.updateScene(scene);

    scene->getScene()->setActiveCollisionDetector(MyCollisionDetectorAllocatorBullet::create(),
                                                  true);

    ompl::msg::setLogLevel(ompl::msg::LOG_INFO);

    moveit_msgs::MotionPlanRequest request = makeAppleReachRequest(drone, tree_scene.apples, "RRTX", 60.0);



    rviz.addGoalMarker("goal_request_marker", request);

    rviz.updateMarkers();

    auto simple_planner = init_planner(drone, scene);

    auto response = simple_planner->plan(scene, request);

    if (response.error_code_.val == 1) {
        rviz.updateTrajectory(response.trajectory_);
    }

    std::cin.get();

    return 0;
}


