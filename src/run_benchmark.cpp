#include <robowflex_library/builder.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_ompl/ompl_interface.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>
#include <robowflex_library/log.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <random_numbers/random_numbers.h>
#include <Eigen/Geometry>
#include "procedural_tree_generation.h"
#include "build_request.h"
#include "EndEffectorConstraintSampler.h"

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
    auto tree_scene = establishPlanningScene();
    scene->getScene()->setPlanningSceneDiffMsg(tree_scene.moveit_diff);

    auto simple_planner = std::make_shared<OMPL::OMPLInterfacePlanner>(drone, "simple");

    OMPL::Settings settings;
    settings.simplify_solutions = false;
    // FIXME: see https://github.com/KavrakiLab/robowflex/issues/239
    // settings.max_goal_samples = 1;

    if (!simple_planner->initialize("package://drone_moveit_config/config/ompl_planning.yaml", settings)) {
        std::cout << "Planner initialization failed." << std::endl;
        return 1;
    }

    simple_planner->getInterface()
    .getConstraintSamplerManager()
    .registerSamplerAllocator(std::make_shared<EndEffectorPositionConstraintSamplerAllocator>());

    Profiler::Options options;
    options.metrics = Profiler::WAYPOINTS | Profiler::CORRECT | Profiler::LENGTH | Profiler::SMOOTHNESS;
    Experiment experiment("pick_apple", options, 10.0, 5);

    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

    log::showUpToWarning();

    for (const auto &config : {"PRM", "RRTConnect", "RRTStar", "TRRT"}) {
        for (int i = 0; i < 5; i++) {
           experiment.addQuery(config, scene, simple_planner,
                               makeAppleReachRequest(drone, tree_scene.apples, config));
        }
    }

    auto dataset = experiment.benchmark(1);

    OMPLPlanDataSetOutputter output("robowflex_drone_ompl");
    output.dump(*dataset);

    return 0;
}
