#include <robowflex_library/builder.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_ompl/ompl_interface.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/io/broadcaster.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <random_numbers/random_numbers.h>
#include "procedural_tree_generation.h"
#include "EndEffectorConstraintSampler.h"
#include "build_request.h"

using namespace robowflex;

int main(int argc, char **argv) {

    // Startup ROS
    ROS ros(argc, argv);

    auto drone = std::make_shared<Robot>("drone");

    drone->initialize(
            "package://drone_moveit_config/urdf/bot.urdf",
            "package://drone_moveit_config/config/aerial_manipulator_drone.srdf",
            "",
            ""
    );

    IO::RVIZHelper rviz(drone);
    IO::RobotBroadcaster bc(drone);
    bc.start();

    auto scene = std::make_shared<Scene>(drone);
    auto tree_scene = establishPlanningScene();
    scene->getScene()->setPlanningSceneDiffMsg(tree_scene.moveit_diff);

    rviz.updateScene(scene);

    auto simple_planner = std::make_shared<OMPL::OMPLInterfacePlanner>(drone, "simple");

    OMPL::Settings settings;
    settings.simplify_solutions = false;

    if (!simple_planner->initialize("package://drone_moveit_config/config/ompl_planning.yaml", settings)) {
        std::cout << "Planner initialization failed." << std::endl;
        return 1;
    }

    simple_planner->getInterface()
            .getConstraintSamplerManager()
            .registerSamplerAllocator(std::make_shared<EndEffectorPositionConstraintSamplerAllocator>());

    Profiler::Options options;
    options.metrics = Profiler::WAYPOINTS | Profiler::CORRECT | Profiler::LENGTH | Profiler::SMOOTHNESS | Profiler::CLEARANCE;
    Experiment experiment("pick_apple", options, 10.0, 10);

    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

    moveit_msgs::MotionPlanRequest request = makeAppleReachRequest(drone, tree_scene.apples, "TRRT");

    rviz.addGoalMarker("goal_request_marker", request);

    rviz.updateMarkers();

    auto response = simple_planner->plan(scene, request);

    if (response.error_code_.val == 1) {
        rviz.updateTrajectory(response.trajectory_);
    }

    std::cin.get();

    return 0;
}

