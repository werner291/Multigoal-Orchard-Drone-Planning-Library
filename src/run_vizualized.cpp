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
#include <fcl/fcl.h>
#include <ompl/geometric/planners/prm/PRM.h>

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

    si->setup();

    full_trajectory.addSuffixWaypoint(genStartState(drone));

    ompl::geometric::PRM prm(si);

    auto avoid_branches = std::make_shared<InverseClearanceIntegralObjectiveOMPL>(si, false);

    std::cout << "Building PRM" << std::endl;

    for (const Apple &apple: tree_scene.apples) {

        ompl::base::ScopedState start(si);
        state_space->copyToOMPLState(start.get(), full_trajectory.getTrajectory()->getLastWayPoint());

        auto pdef = std::make_shared<ompl::base::ProblemDefinition>(si);
        pdef->addStartState(start);
//        pdef->setOptimizationObjective(avoid_branches);

        pdef->setGoal(std::make_shared<DroneEndEffectorNearTarget>(si, 0.2, apple.center));

        prm.setProblemDefinition(pdef);

        std::chrono::steady_clock::time_point pre_solve = std::chrono::steady_clock::now();
        ompl::base::PlannerStatus status = prm.solve(ompl::base::timedPlannerTerminationCondition(5.0));
        std::chrono::steady_clock::time_point post_solve = std::chrono::steady_clock::now();

        if (status) {
            auto path = pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();
            for (auto state: path->getStates()) {
                state_space->copyToRobotState(*drone->getScratchState(), state);
                full_trajectory.addSuffixWaypoint(*drone->getScratchState());
            }
            std::cout << "Point-to-point solution found in " << std::chrono::duration_cast<std::chrono::milliseconds>((post_solve - pre_solve)).count() << "ms" << std::endl;
        } else {
            std::cout << "Apple unreachable" << std::endl;
        }

        prm.clearQuery();
    }

    full_trajectory.interpolate(20 * full_trajectory.getNumWaypoints());

    rviz.updateTrajectory(full_trajectory);

    return 0;
}


