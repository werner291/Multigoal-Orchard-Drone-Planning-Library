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
#include "LeavesCollisionChecker.h"
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

        LeavesCollisionChecker leavesCollisionChecker(tree_scene.leaf_vertices);

        const std::shared_ptr<ompl::base::SpaceInformation> si = initSpaceInformation(scene, drone, state_space);
        const robot_state::RobotState start_state = genStartState(drone);

        std::vector<std::shared_ptr<MultiGoalPlanner>> multiplanners{
                std::make_shared<KNNPlanner>(1),
                std::make_shared<KNNPlanner>(2),
                std::make_shared<KNNPlanner>(3),
                std::make_shared<KNNPlanner>(5),
                std::make_shared<RandomPlanner>()
        };

        for (const auto &planner: multiplanners) {

            std::vector<std::shared_ptr<ompl::base::Planner>> subplanners{
                    std::make_unique<ompl::geometric::PRM>(si)//, std::make_unique<ompl::geometric::RRTConnect>(si)
            };

            // Nesting order is important here because the sub-planners are re-created every run.
            for (auto &sub_planner: subplanners) {
                std::cout << "Attempting " << planner->getName() << " with sub-planner " << sub_planner->getName()
                          << std::endl;

                MultiGoalPlanResult result = planner->plan(tree_scene.apples, start_state, scene, drone, *sub_planner);

                result.stats["is_collision_free"] = result.trajectory.isCollisionFree(scene);

                std::set<size_t> leaves;
                size_t unique_collisions = 0;

                for (double t = 0.0;
                     t < result.trajectory.getTrajectory()->getDuration(); t += 0.1) { // NOLINT(cert-flp30-c)
                    result.trajectory.getTrajectory()->getStateAtDurationFromStart(t, drone->getScratchState());
                    std::set<size_t> new_leaves = leavesCollisionChecker.checkLeafCollisions(*drone->getScratchState());
                    std::set<size_t> added_leaves;
                    std::set_difference(new_leaves.begin(), new_leaves.end(), leaves.begin(), leaves.end(),
                                        std::inserter(added_leaves, added_leaves.end()));
                    std::set<size_t> removed_leaves;
                    std::set_difference(leaves.begin(), leaves.end(), new_leaves.begin(), new_leaves.end(),
                                        std::inserter(removed_leaves, removed_leaves.end()));
                    unique_collisions += added_leaves.size();

                    if (!added_leaves.empty() || !removed_leaves.empty()) {
                        Json::Value leaf_collisions;
                        leaf_collisions["t"] = t;
                        leaf_collisions["contacts_ended"] = (int) removed_leaves.size();
                        leaf_collisions["new_leaves_in_contact"] = (int) added_leaves.size();
                        result.stats["leaf_collisions_over_time"].append(leaf_collisions);
                    }

                    leaves = new_leaves;

                }

                result.stats["unique_leaves_collided"] = (int) unique_collisions;

                result.stats["intermediate_planner"] = sub_planner->getName();
                run_results["planner_runs"].append(result.stats);

            }
        }

        benchmark_results.append(run_results);
    }

    std::ofstream results(RUNS < 50 ? "analysis/results_test.json" : "analysis/results.json");
    results << benchmark_results;
    results.close();

    std::cout << "Done!" << std::endl;

    return 0;
}