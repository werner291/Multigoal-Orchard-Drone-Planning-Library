#include <robowflex_library/builder.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_ompl/ompl_interface.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/io/broadcaster.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <random_numbers/random_numbers.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include "EndEffectorConstraintSampler.h"
#include "build_request.h"
#include "build_planning_scene.h"
#include "ClearanceDecreaseMinimzationObjective.h"
#include "make_robot.h"
#include "BulletContinuousMotionValidator.h"
#include "init_planner.h"

using namespace robowflex;

int main(int argc, char **argv) {

    // Startup ROS
    ROS ros(argc, argv);

    std::shared_ptr<Robot> drone = make_robot();

    IO::RVIZHelper rviz(drone);
    IO::RobotBroadcaster bc(drone);
    bc.start();

    auto scene = std::make_shared<Scene>(drone);
    auto tree_scene = establishPlanningScene();
    scene->getScene()->setPlanningSceneDiffMsg(tree_scene.moveit_diff);

    rviz.updateScene(scene);

    scene->getScene()->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(),
                                                  true);

    ompl::msg::setLogLevel(ompl::msg::LOG_INFO);

    moveit_msgs::MotionPlanRequest request = makeAppleReachRequest(drone, tree_scene.apples, "BiTRRT");

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


