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
#include "multi_goal_planners.h"
#include "ompl_custom.h"
#include <fcl/fcl.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
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

//    IO::RVIZHelper rviz(drone);
//    IO::RobotBroadcaster bc(drone);
//    bc.start();

    const int RUNS = 100;
    Json::Value benchmark_results;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> apple_count(5, 100);

    ompl::msg::setLogLevel(ompl::msg::LOG_INFO);
    ompl_interface::ModelBasedStateSpaceSpecification spec(drone->getModelConst(), "whole_body");
    auto state_space = std::make_shared<DroneStateSpace>(spec);

    for (int i = 0; i < RUNS; i++) {

        Json::Value run_results;

        auto scene = std::make_shared<Scene>(drone);
        int numberOfApples = apple_count(gen);

        run_results["number_of_apples"] = numberOfApples;

        auto tree_scene = establishPlanningScene(10, numberOfApples);
        scene->getScene()->setPlanningSceneDiffMsg(tree_scene.moveit_diff);
        // Diff message apparently can't handle this?
        scene->getScene()->getAllowedCollisionMatrixNonConst().setDefaultEntry("leaves", true);
        scene->getScene()->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

        const std::shared_ptr<ompl::base::SpaceInformation> si = initSpaceInformation(scene, drone, state_space);
        {
            ompl::geometric::PRM prm(si);
            MultiGoalPlanResult result_random_prm = plan_random(tree_scene.apples, genStartState(drone), scene, drone,
                                                                prm);

            result_random_prm.stats["intermediate_planner"] = "PRM";
            run_results["planner_runs"].append(result_random_prm.stats);
        }
        {
            ompl::geometric::RRTConnect rrtconnect(si);
            MultiGoalPlanResult result_random_rrtconnect = plan_random(tree_scene.apples, genStartState(drone), scene,
                                                                       drone, rrtconnect);

            result_random_rrtconnect.stats["intermediate_planner"] = "RRTConnect";
            run_results["planner_runs"].append(result_random_rrtconnect.stats);
        }
        {
            ompl::geometric::PRM prm(si);
            MultiGoalPlanResult result_random_prm = plan_nn_rrtconnect(tree_scene.apples, genStartState(drone), scene,
                                                                       drone, prm);

            result_random_prm.stats["intermediate_planner"] = "PRM";
            run_results["planner_runs"].append(result_random_prm.stats);
        }
        {
            ompl::geometric::RRTConnect rrtconnect(si);
            MultiGoalPlanResult result_random_rrtconnect = plan_nn_rrtconnect(tree_scene.apples, genStartState(drone),
                                                                              scene, drone, rrtconnect);

            result_random_rrtconnect.stats["intermediate_planner"] = "RRTConnect";
            run_results["planner_runs"].append(result_random_rrtconnect.stats);
        }

        benchmark_results.append(run_results);
    }

    std::ofstream results("analysis/results.json");
    results << benchmark_results;
    results.close();

    return 0;
}