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
    scene->getScene()->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

    rviz.updateScene(scene);

    ompl::msg::setLogLevel(ompl::msg::LOG_INFO);

    std::vector<planning_interface::MotionPlanResponse> responses;

    Trajectory full_trajectory(drone, "whole_body");
    full_trajectory.addSuffixWaypoint(genStartState(drone));

    ompl_interface::ModelBasedStateSpaceSpecification spec(drone->getModel(), "whole_body");

    auto state_space = std::make_shared<CustomModelBasedStateSpace>(spec);
    state_space->setStateSamplerAllocator([](const ompl::base::StateSpace *space) {
        return std::make_shared<DroneStateSampler>(space);
    });

    auto si = std::make_shared<ompl::base::SpaceInformation>(state_space);
    si->setStateValidityChecker(std::make_shared<StateValidityChecker>(si.get(), scene));
    si->setMotionValidator(std::make_shared<BulletContinuousMotionValidator>(si.get(), drone, scene));
    si->setup();

    full_trajectory.addSuffixWaypoint(genStartState(drone));

    ompl::geometric::PRM prm(si);

    auto avoid_branches = std::make_shared<InverseClearanceIntegralObjectiveOMPL>(si, false);

    Json::Value root;

    for (const Apple &apple: tree_scene.apples) {

        Json::Value apple_statpoint;

        apple_statpoint["apple"][0] = apple.center.x();
        apple_statpoint["apple"][1] = apple.center.y();
        apple_statpoint["apple"][2] = apple.center.z();

        ompl::base::ScopedState start(si);
        state_space->copyToOMPLState(start.get(), full_trajectory.getTrajectory()->getLastWayPoint());

        auto pdef = std::make_shared<ompl::base::ProblemDefinition>(si);
        pdef->addStartState(start);
//        pdef->setOptimizationObjective(avoid_branches);

        auto goal = std::make_shared<DroneEndEffectorNearTarget>(si, 0.2, apple.center);
        pdef->setGoal(goal);

        prm.setProblemDefinition(pdef);

        std::chrono::steady_clock::time_point pre_solve = std::chrono::steady_clock::now();
        ompl::base::PlannerStatus status = prm.solve(ompl::base::timedPlannerTerminationCondition(5.0));
        std::chrono::steady_clock::time_point post_solve = std::chrono::steady_clock::now();

        long elapsed_millis = std::chrono::duration_cast<std::chrono::milliseconds>(
                (post_solve - pre_solve)).count();

        ompl::geometric::PathSimplifier ps(si);

        if (status == ompl::base::PlannerStatus::EXACT_SOLUTION) {

            auto path = pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();

            ps.shortcutPath(*path, 50);
            ps.smoothBSpline(*path);

            for (auto state: path->getStates()) {
                state_space->copyToRobotState(*drone->getScratchState(), state);
                full_trajectory.addSuffixWaypoint(*drone->getScratchState());

                Json::Value traj_pt;
                for (int i = 0; i < drone->getScratchState()->getVariableCount(); i += 1) {
                    traj_pt["values"][i] = drone->getScratchState()->getVariablePosition(i);
                }
//                traj_pt["clearance"] = si->getStateValidityChecker()->clearance(state);
                apple_statpoint["trajectory"].append(traj_pt);
            }

            std::cout << "Point-to-point solution found in " << elapsed_millis << "ms" << std::endl;

            apple_statpoint["solved"] = true;

        } else {
            apple_statpoint["solved"] = false;
            std::cout << "Apple unreachable" << std::endl;
        }

        apple_statpoint["feasible_solve_milliseconds"] = (int) elapsed_millis;
        apple_statpoint["prm_nodes_after_solve"] = (int) prm.milestoneCount();
        apple_statpoint["prm_edges_after_solve"] = (int) prm.edgeCount();
        apple_statpoint["goal_samples_tried"] = (int) goal->getSamplesTried();
        apple_statpoint["goal_samples_yielded"] = (int) goal->getSamplesYielded();

        root.append(apple_statpoint);

        prm.clearQuery();
    }

    std::cout << "Writing results" << std::endl;

    std::ofstream myfile;
    myfile.open("analysis/results.json");
    myfile << root;
    myfile.close();

    full_trajectory.interpolate(2 * full_trajectory.getNumWaypoints());

    rviz.updateTrajectory(full_trajectory);

    return 0;
}