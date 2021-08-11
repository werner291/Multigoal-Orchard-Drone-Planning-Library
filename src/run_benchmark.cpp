#include <robowflex_library/builder.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_ompl/ompl_interface.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <random_numbers/random_numbers.h>
#include <Eigen/Geometry>
#include "procedural_tree_generation.h"

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

    auto scene = std::make_shared<Scene>(drone);
    //scene->getScene()->setPlanningSceneMsg(establishPlanningScene());

    auto simple_planner = std::make_shared<OMPL::OMPLInterfacePlanner>(drone, "simple");

    OMPL::Settings settings;
    settings.simplify_solutions = false;
    // FIXME: see https://github.com/KavrakiLab/robowflex/issues/239
    // settings.max_goal_samples = 1;

    if (!simple_planner->initialize("package://drone_moveit_config/config/ompl_planning.yaml", settings)) {
        std::cout << "Planner initialization failed." << std::endl;
        return 1;
    }

    Profiler::Options options;
    options.metrics = Profiler::WAYPOINTS | Profiler::CORRECT | Profiler::LENGTH | Profiler::SMOOTHNESS;
    Experiment experiment("pick_apple", options, 10.0, 100);

    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

    for (const auto &config : {"PRM", "RRTConnect", "RRTStar", "TRRT"}) {
        auto request = std::make_shared<MotionRequestBuilder>(simple_planner, "whole_body");


        request->setConfig(config);
        request->setWorkspaceBounds(
                Eigen::Vector3d(-20.0, -20.0, -20.0),
                Eigen::Vector3d(20.0, 20.0, 20.0)
        );

        request->setStartConfiguration({-10.0, -10.0, 10.0, 0.0, 0.0, 0.0, 1.0, 0.5});

        request->setGoalConfiguration({10.0, 10.0, 10.0, 0.0, 0.0, 0.0, 1.0, 0.5});

        experiment.addQuery(config, scene, simple_planner, request);
    }

    auto dataset = experiment.benchmark(1);

    OMPLPlanDataSetOutputter output("robowflex_drone_ompl");
    output.dump(*dataset);

    return 0;
}
