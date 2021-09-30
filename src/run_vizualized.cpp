#include <robowflex_library/builder.h>
#include <robowflex_library/util.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/trajectory.h>
#include "msgs_utilities.h"
#include "build_planning_scene.h"
#include "make_robot.h"
#include "InverseClearanceIntegralObjective.h"
#include "BulletContinuousMotionValidator.h"
#include "multigoal/multi_goal_planners.h"
#include "multigoal/knn.h"
#include "multigoal/uknn.h"
#include "multigoal/random_order.h"
#include "ompl_custom.h"
#include "LeavesCollisionChecker.h"
#include <fcl/fcl.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <json/json.h>

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

    const int RUNS = 50; // 100 Is the value reported in the paper.
    Json::Value benchmark_results;

    std::random_device rd;
    std::mt19937 gen(rd());

    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

    ompl_interface::ModelBasedStateSpaceSpecification spec(drone->getModelConst(), "whole_body");
    auto state_space = std::make_shared<DroneStateSpace>(spec);

    auto scene = std::make_shared<Scene>(drone);

    double apple_t = std::uniform_real_distribution(0.0, 1.0)(gen);

    int numberOfApples = 1 + (apple_t * apple_t) * 99;

    auto tree_scene = establishPlanningScene(10, numberOfApples);
    scene->getScene()->setPlanningSceneDiffMsg(tree_scene.moveit_diff);
    // Diff message apparently can't handle this?
    scene->getScene()->getAllowedCollisionMatrixNonConst().setDefaultEntry("leaves", true);
    scene->getScene()->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

    const std::shared_ptr<ompl::base::SpaceInformation> si = initSpaceInformation(scene, drone, state_space);
    const robot_state::RobotState start_state = genStartState(drone);

    auto multiplanner = std::make_shared<KNNPlanner>(1);
    auto sub_planner = std::make_unique<ompl::geometric::PRM>(si);

                MultiGoalPlanResult result = multiplanner->plan(tree_scene.apples, start_state, scene, drone,
                                                                *sub_planner);

    rviz.updateTrajectory(result.trajectory);

    return 0;
}