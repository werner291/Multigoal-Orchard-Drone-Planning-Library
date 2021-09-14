#include <robowflex_library/builder.h>
#include <robowflex_library/util.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/trajectory.h>
#include "msgs_utilities.h"
#include "build_planning_scene.h"
#include "make_robot.h"
#include "init_planner.h"
#include "InverseClearanceIntegralObjective.h"
#include "ompl_custom.h"
#include "BulletContinuousMotionValidator.h"
#include "multi_goal_planners.h"
#include <fcl/fcl.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <json/json.h>

static const int NUM_APPLES = 20;
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
    auto tree_scene = establishPlanningScene(10, NUM_APPLES);
    scene->getScene()->setPlanningSceneDiffMsg(tree_scene.moveit_diff);
    // Diff message apparently can't handle this?
    scene->getScene()->getAllowedCollisionMatrixNonConst().setDefaultEntry("leaves", true);
    scene->getScene()->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

    rviz.updateScene(scene);

    ompl::msg::setLogLevel(ompl::msg::LOG_INFO);

    std::vector<planning_interface::MotionPlanResponse> responses;

    MultiGoalPlanResult result = plan_random(
            tree_scene.apples,
            genStartState(drone),
            scene,
            drone
    );

    Trajectory full_trajectory = result.trajectory;

    full_trajectory.interpolate(2 * full_trajectory.getNumWaypoints());

    rviz.updateTrajectory(full_trajectory);

    return 0;
}