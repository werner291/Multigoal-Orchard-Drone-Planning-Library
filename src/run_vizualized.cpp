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
#include "multigoal/approach_clustering.h"
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <json/json.h>

using namespace robowflex;
using namespace multigoal;

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

    int numberOfApples = 100;

    auto tree_scene = establishPlanningScene(10, numberOfApples);
    scene->getScene()->setPlanningSceneDiffMsg(tree_scene.moveit_diff);
    // Diff message apparently can't handle this?
    scene->getScene()->getAllowedCollisionMatrixNonConst().setDefaultEntry("leaves", true);
    scene->getScene()->getAllowedCollisionMatrixNonConst().setDefaultEntry("apples", true);
    scene->getScene()->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());
    rviz.updateScene(scene);

    const std::shared_ptr<ompl::base::SpaceInformation> si = initSpaceInformation(scene, drone, state_space);
    const robot_state::RobotState start_state = genStartState(drone);

    auto leavesCollisionChecker = std::make_shared<LeavesCollisionChecker>(tree_scene.leaf_vertices);
    auto leafCountObjective = std::make_shared<LeavesCollisionCountObjective>(si, drone->getModelConst(),
                                                                              leavesCollisionChecker);

    auto approach_table = takeGoalSamples(si, ApproachClustering::constructGoalRegions(tree_scene, si), 50);

    keepBest(*leafCountObjective, approach_table, 5);

    rviz.addMarker(buildApproachTableVisualization(drone, approach_table));
    rviz.updateMarkers();


    //
//    auto multiplanner = std::make_shared<KNNPlanner>(1);
//    auto sub_planner = std::make_shared<ompl::geometric::PRMstar>(si);
//
//    auto opt = std::make_shared<LeavesCollisionCountObjective>(
//            si,
//            drone->getModelConst(),
//            leavesCollisionChecker);
//
//    PointToPointPlanner ptp(sub_planner,opt,drone);
//
//    MultiGoalPlanResult result = multiplanner->plan(tree_scene, start_state, scene, drone, ptp);
//
//    result.trajectory.interpolate(10000);
//
//    rviz.updateTrajectory(result.trajectory);


    return 0;
}