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


    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

    ompl_interface::ModelBasedStateSpaceSpecification spec(drone->getModelConst(), "whole_body");
    auto state_space = std::make_shared<DroneStateSpace>(spec);

    for (int i = 0; i < RUNS; i++) {


        Json::Value run_results;

        auto scene = std::make_shared<Scene>(drone);

        double apple_t = std::uniform_real_distribution(0.0,1.0)(gen);

        int numberOfApples = 1 + (apple_t * apple_t) * 99;

        std::cout << "Run " << (i+1) << " out of " << RUNS << " with " << numberOfApples << " apples." << std::endl;


        run_results["number_of_apples"] = numberOfApples;

        auto tree_scene = establishPlanningScene(10, numberOfApples);
        scene->getScene()->setPlanningSceneDiffMsg(tree_scene.moveit_diff);
        // Diff message apparently can't handle this?
        scene->getScene()->getAllowedCollisionMatrixNonConst().setDefaultEntry("leaves", true);
        scene->getScene()->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

        const std::shared_ptr<ompl::base::SpaceInformation> si = initSpaceInformation(scene, drone, state_space);
        const robot_state::RobotState start_state = genStartState(drone);



        {
            std::cout << "Attempting PRM with random order." << std::endl;
            ompl::geometric::PRM prm(si);
            MultiGoalPlanResult result_random_prm = plan_random(tree_scene.apples, start_state, scene, drone,
                                                                prm);

            result_random_prm.stats["intermediate_planner"] = "PRM";
            run_results["planner_runs"].append(result_random_prm.stats);
        }
        {
            std::cout << "Attempting RRTConnect with random order." << std::endl;
            ompl::geometric::RRTConnect rrtconnect(si);
            MultiGoalPlanResult result_random_rrtconnect = plan_random(tree_scene.apples, start_state, scene,
                                                                       drone, rrtconnect);

            result_random_rrtconnect.stats["intermediate_planner"] = "RRTConnect";
            run_results["planner_runs"].append(result_random_rrtconnect.stats);
        }
        {
            std::cout << "Attempting PRM nearest neighbor." << std::endl;
            ompl::geometric::PRM prm(si);
            MultiGoalPlanResult result_random_prm = plan_nn(tree_scene.apples, start_state, scene,
                                                            drone, prm);

            result_random_prm.stats["intermediate_planner"] = "PRM";
            run_results["planner_runs"].append(result_random_prm.stats);
        }
        {
            std::cout << "Attempting RRTConnect nearest neighbor." << std::endl;
            ompl::geometric::RRTConnect rrtconnect(si);
            MultiGoalPlanResult result_random_rrtconnect = plan_nn(tree_scene.apples, start_state,
                                                                   scene, drone, rrtconnect);

            result_random_rrtconnect.stats["intermediate_planner"] = "RRTConnect";
            run_results["planner_runs"].append(result_random_rrtconnect.stats);
        }
        {
            std::cout << "Attempting PRM 5-nearest neighbor." << std::endl;

            ompl::geometric::PRM prm(si);
            MultiGoalPlanResult result_random_prm = plan_knn(tree_scene.apples, start_state, scene,
                                                            drone, 5, prm);

            result_random_prm.stats["intermediate_planner"] = "PRM";
            run_results["planner_runs"].append(result_random_prm.stats);
        }
        {
            std::cout << "Attempting RRTConnect 5-nearest neighbor." << std::endl;

            ompl::geometric::RRTConnect rrtconnect(si);
            MultiGoalPlanResult result_random_rrtconnect = plan_knn(tree_scene.apples, start_state,
                                                                   scene, drone, 5, rrtconnect);

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